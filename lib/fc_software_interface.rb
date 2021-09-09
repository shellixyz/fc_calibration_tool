
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

    def self.autodetect serial_device, baud = 115200
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
                return INavInterface.new msp
            end
        rescue MSP::ProtoError::ReadTimeout, MSP::ProtoError::SyncFailed
        end

        begin
            mavlink = Mavlink.new serial_device, baud
            mavlink.set_message_interval :SYS_STATUS, 0.02
            mavlink.wait_for_message :SYS_STATUS
            return ArdupilotInterface.new mavlink
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

    def initialize serial_device, baud = 115200
        @mavlink = @proto_interface = serial_device.is_a?(Mavlink) ? serial_device : Mavlink.new(serial_device, baud)
        @sys_status_message_interval_configured = false
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

    attr_reader :mavlink

end

class INavInterface < FCSoftwareInterface

    def target_software
        :inav
    end

    def initialize serial_device, baud = 115200
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
