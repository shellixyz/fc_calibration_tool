
require 'forwardable'
require_relative 'mavlink-ruby/mavlink'
require_relative 'msp-ruby/msp'

class FCSoftwareInterface

    VoltageAndCurrent = Struct.new :voltage, :current
    CurrentOffsetAndScale = Struct.new :offset, :scale

    module Error

        class BatterySensorNotSupported < StandardError; end
        class BatterySensorNotEnabled < StandardError; end
        class UnsupportedFirmware < StandardError; end
        class FirmwareAutodetectionFailed < StandardError; end
        class ProtocolError < StandardError; end

    end

    def self.unsupported_firmware firmware
        raise Error::UnsupportedFirmware, "Firmware not supported (only iNav is supported): #{firmware}"
    end

    def self.get software
        case software
        when :ardupilot then ArdupilotInterface
        when :inav then INavInterface
        else raise ArgumentError, "unsupported software: #{software}"
        end
    end

    def self.autodetect serial_device, baud = 115200, battery: nil
        begin
            msp = MSP.new serial_device, baud
            tries = 1
            begin
                firmware = msp.command :fc_variant
            rescue MSP::ProtoError
                if tries < 4
                    tries += 1
                    retry
                end
                raise
            end
            if firmware == 'ARDU'
                msp.reboot_fc resume: false
                sleep 1
            else
                return INavInterface.new msp, battery: battery
            end
        rescue MSP::ProtoError::ReadTimeout, MSP::ProtoError::SyncFailed
        end

        begin
            mavlink = Mavlink.new serial_device, baud
            mavlink.set_message_interval :SYS_STATUS, 0.02
            mavlink.wait_for_message :SYS_STATUS
            return ArdupilotInterface.new mavlink, battery: battery
        rescue Mavlink::Error::Timeout
        end

        raise Error::FirmwareAutodetectionFailed, 'firmware auto-detection failed'
    end

    def sample_current
        sample_voltage_and_current.current
    end

    def sample_voltage
        sample_voltage_and_current.voltage
    end

    def prepare_current_calibration
        set_current_offset_and_scale 0, current_measurement_scale
    end

    def prepare_voltage_calibration
        set_voltage_scale voltage_measurement_scale
    end

    def needs_reboot?
        @needs_reboot
    end

    extend Forwardable
    def_delegators :proto_interface, :reboot_fc

    attr_reader :proto_interface

end

FCSI = FCSoftwareInterface

class ArdupilotInterface < FCSoftwareInterface

    def target_software
        :ardupilot
    end

    def initialize serial_device, baud = 115200, battery: nil
        @mavlink = @proto_interface = serial_device.is_a?(Mavlink) ? serial_device : Mavlink.new(serial_device, baud)
        @autopilot = detect_autopilot
        @battery_name = battery
        @sys_status_message_interval_configured = false
        @battery_status_message_interval_configured = false
    end

    def voltage_sensor_present?
        battery_sensor_present?
    end

    def current_sensor_present?
        battery_sensor_present?
    end

    def voltage_sensor_enabled?
        [ 3, 4 ].include? mavlink.param_value(:BATT_MONITOR)
    end

    def current_sensor_enabled?
        mavlink.param_value(:BATT_MONITOR) == 4
    end

    def enable_voltage_sensor
        current_param_value = mavlink.param_value :BATT_MONITOR
        raise "The BATT_MONITOR param is not 0" unless current_param_value == 0
        mavlink.set_param_value :BATT_MONITOR, 3 unless current_param_value == 4
        @needs_reboot = true
    end

    def enable_current_sensor
        current_param_value = mavlink.param_value :BATT_MONITOR
        raise "The BATT_MONITOR param is different from 0 or 3" unless [0, 3].include? current_param_value
        mavlink.set_param_value :BATT_MONITOR, 4 unless current_param_value == 4
        @needs_reboot = true
    end

    def prepare_current_calibration
        super
        configure_sys_status_message_interval
    end

    def prepare_voltage_calibration
        super
        configure_sys_status_message_interval
    end

    def sample_voltage_and_current
        return sample_px4_voltage_and_current if px4?

        configure_sys_status_message_interval
        mavlink.wait_for_message :SYS_STATUS do |message|
            voltage = message.content[:voltage_battery] / 1000.0
            current = message.content[:current_battery] / 100.0
            FCSI::VoltageAndCurrent.new voltage, current
        end
    end

    def voltage_scale
        mavlink.param_value :BATT_VOLT_MULT
    end

    def set_voltage_scale scale
        mavlink.set_param_value :BATT_VOLT_MULT, scale
    end

    def current_offset_and_scale
        offset = mavlink.param_value :BATT_AMP_OFFSET
        scale = mavlink.param_value :BATT_AMP_PERVLT
        FCSI::CurrentOffsetAndScale.new offset, scale
    end

    def set_current_offset_and_scale offset, scale
        mavlink.set_param_value :BATT_AMP_OFFSET, offset
        mavlink.set_param_value :BATT_AMP_PERVLT, scale
        nil
    end

    def current_measurement_scale
        100
    end

    def voltage_measurement_scale
        20
    end

    def save_settings
        # Ardupilot saves the params as soon as they are set
    end

    def reboot_fc
        super
        @sys_status_message_interval_configured = false
    end

    private

    def detect_autopilot
        mavlink.wait_for_message(:HEARTBEAT) { |message| message.content[:autopilot] }
    rescue Mavlink::Error::Timeout
        nil
    end

    def px4?
        @autopilot == :MAV_AUTOPILOT_PX4
    end

    def sample_px4_voltage_and_current
        configure_battery_status_message_interval

        battery_id = px4_display_battery_id
        raise Error::BatterySensorNotSupported, "Unable to resolve PX4 #{@battery_name}" if @battery_name and battery_id.nil?

        message =
            if battery_id.nil?
                mavlink.wait_for_message(:BATTERY_STATUS) { |packet| packet.content }
            else
                mavlink.wait_for_message(:BATTERY_STATUS, id: battery_id) { |packet| packet.content }
            end

        voltage_raw = message[:voltages].is_a?(Array) ? message[:voltages].first : message[:voltages]
        voltage = voltage_raw == 65535 ? Float::NAN : voltage_raw / 1000.0
        current_raw = message[:current_battery]
        current = current_raw == 65535 ? Float::NAN : current_raw / 100.0
        FCSI::VoltageAndCurrent.new voltage, current
    end

    def px4_display_battery_id
        return @px4_display_battery_id if defined?(@px4_display_battery_id)

        @px4_display_battery_id =
            if @battery_name
                select_px4_battery_id requested_px4_battery_slot
            elsif px4_bat_current_only?(1)
                select_px4_current_only_battery_id
            end
    end

    def requested_px4_battery_slot
        @requested_px4_battery_slot ||= @battery_name.delete_prefix('BAT').to_i
    end

    def px4_bat_current_only? slot
        mavlink.param_value("BAT#{slot}_V_CHANNEL".intern) == -1 && mavlink.param_value("BAT#{slot}_I_CHANNEL".intern) >= 0
    rescue Mavlink::FailedToGetParam
        false
    end

    def px4_bat_voltage_only? slot
        mavlink.param_value("BAT#{slot}_V_CHANNEL".intern) >= 0 && mavlink.param_value("BAT#{slot}_I_CHANNEL".intern) == -1
    rescue Mavlink::FailedToGetParam
        false
    end

    def px4_bat_voltage_and_current? slot
        mavlink.param_value("BAT#{slot}_V_CHANNEL".intern) >= 0 && mavlink.param_value("BAT#{slot}_I_CHANNEL".intern) >= 0
    rescue Mavlink::FailedToGetParam
        false
    end

    def select_px4_battery_id slot
        samples = collect_px4_battery_samples
        return nil if samples.empty?

        selected =
            if px4_bat_current_only?(slot)
                samples.min_by { |sample| [sample[:current_ca].abs, sample[:voltage_mv]] }
            elsif px4_bat_voltage_only?(slot) || px4_bat_voltage_and_current?(slot)
                samples.max_by { |sample| [sample[:voltage_mv], -sample[:current_ca].abs] }
            end

        selected&.fetch(:id, nil)
    end

    def collect_px4_battery_samples
        samples = Hash.new { |hash, key| hash[key] = [] }

        20.times do
            message = mavlink.wait_for_message(:BATTERY_STATUS) { |packet| packet.content }
            next if message[:current_battery] == 65535

            samples[message[:id]] << message
        end

        samples.map do |battery_id, messages|
            voltage_values = messages.filter_map do |message|
                voltage = message[:voltages].is_a?(Array) ? message[:voltages].first : message[:voltages]
                voltage unless voltage == 65535
            end
            current_values = messages.map { |message| message[:current_battery] }

            {
                id: battery_id,
                voltage_mv: voltage_values.empty? ? 65535 : voltage_values.sum / voltage_values.length.to_f,
                current_ca: current_values.sum / current_values.length.to_f
            }
        end
        .reject { |sample| sample[:voltage_mv] == 65535 }
    rescue Mavlink::Error::Timeout
        []
    end

    def select_px4_current_only_battery_id
        collect_px4_battery_samples.min_by { |sample| [sample[:current_ca].abs, sample[:voltage_mv]] }&.fetch(:id, nil)
    end

    def battery_sensor_present?
        configure_sys_status_message_interval
        mavlink.wait_for_message(:SYS_STATUS) { |message| message.content[:onboard_control_sensors_present].include? :MAV_SYS_STATUS_SENSOR_BATTERY }
    end

    def configure_sys_status_message_interval
        unless sys_status_message_interval_configured?
            mavlink.set_message_interval :SYS_STATUS, 0.02
            @sys_status_message_interval_configured = true
        end
    end

    def sys_status_message_interval_configured?
        @sys_status_message_interval_configured
    end

    def configure_battery_status_message_interval
        unless battery_status_message_interval_configured?
            begin
                mavlink.set_message_interval :BATTERY_STATUS, 0.02
            rescue Mavlink::Error::Timeout
                mavlink.wait_for_message :BATTERY_STATUS
            end
            @battery_status_message_interval_configured = true
        end
    end

    def battery_status_message_interval_configured?
        @battery_status_message_interval_configured
    end

    attr_reader :mavlink

end

class INavInterface < FCSoftwareInterface

    def target_software
        :inav
    end

    def initialize serial_device, baud = 115200, battery: nil
        @msp = @proto_interface = serial_device.is_a?(MSP) ? serial_device : MSP.new(serial_device, baud)
        check_software_type
    end

    def check_software_type
        firmware = msp.command :fc_variant
        self.class.unsupported_firmware firmware if firmware != 'INAV'
    end

    def voltage_sensor_present?
        true # We don't have a way to detect whether the board has a voltage sensor
    end

    def current_sensor_present?
        true # We don't have a way to detect whether the board has a current sensor
    end

    def voltage_sensor_enabled?
        #features.include? :vbat
        msp.feature_enabled? :vbat
    end

    def current_sensor_enabled?
        msp.feature_enabled? :current_meter
    end

    def enable_voltage_sensor
        msp.enable_feature :vbat
        @needs_reboot = true
    end

    def enable_current_sensor
        msp.enable_feature :current_meter
        @needs_reboot = true
    end

    def sample_voltage_and_current
        data = msp.command :inav_analog
        FCSI::VoltageAndCurrent.new data.battery_voltage, data.current
    end

    def voltage_scale
        msp.command(:inav_battery_config).voltage_scale
    end

    def set_voltage_scale scale
        battery_config = msp.command :inav_battery_config
        battery_config.voltage_scale = scale
        msp.command :inav_set_battery_config, *battery_config.to_a

    end

    def current_offset_and_scale
        data = msp.command :inav_battery_config
        offset = data.current_sensor_offset
        scale = data.current_sensor_scale
        FCSI::CurrentOffsetAndScale.new offset, scale
    end

    def set_current_offset_and_scale offset, scale
        battery_config = msp.command :inav_battery_config
        battery_config.current_sensor_offset = offset
        battery_config.current_sensor_scale = scale
        msp.command :inav_set_battery_config, *battery_config.to_a
        nil
    end

    def current_measurement_scale
        0.01
    end

    def voltage_measurement_scale
        20
    end

    def save_settings
        msp.save_settings
    end

    private

    attr_reader :msp

end

if $0 == __FILE__
    port = ARGV[0] or '/dev/ttyACM0'
    baud = ARGV[1] or 115200
    interface = FCSoftwareInterface.autodetect port, baud
    require 'pry'
    interface.pry
end
