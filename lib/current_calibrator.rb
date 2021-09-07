
require 'forwardable'
require_relative 'libutil/lib/term'
require_relative 'fc_software_interface'
require_relative 'current_scale_and_offset_calculator'

class CurrentCalibrator

    class AcquisitionAborted < StandardError; end
    class TooFewMeasurements < StandardError; end
    class InvalidResults < StandardError; end

    DEFAULT_ACQUISITION_TIME = 5 # seconds

    def initialize fc_software_interface, acquisition_time: DEFAULT_ACQUISITION_TIME
        @acquisition_time = acquisition_time
        @fc_measurements = Array.new
        @true_measurements = Array.new
        @fc_software_interface = fc_software_interface
    end

    def run
        check_sensor_enabled
        print '*' * 20 + ' CURRENT CALIBRATION ' + '*' * 20 + "\n\n"
        read_previous_offset_and_scale
        fc_software_interface.prepare_current_calibration
        gather_at_least_2_measurements
        calculate_offset_and_scale
        check_results_sanity
        display_results
        save_calibration_onto_fc
    rescue InvalidResults
        puts
        restore_previous_offset_and_scale_onto_fc
        raise
    rescue Interrupt, TooFewMeasurements
        puts
        restore_previous_offset_and_scale_onto_fc
        STDERR.puts "Current calibration aborted"
    end

    def check_sensor_enabled
        raise "Current sensor not enabled" unless fc_software_interface.current_sensor_enabled?
    end

    def calibration_data
        raise "no data available" if offset_and_scale_calculator.nil?
        data = %i{ offset scale voltage_offset native_voltage_offset native_scale }.map { |value_name| [ value_name, send(value_name) ] }.to_h
        data[:measurements] = true_measurements.zip(fc_measurements).to_h
    end

    extend Forwardable
    def_delegators :offset_and_scale_calculator, :offset, :scale, :voltage_offset, :native_voltage_offset, :native_scale
    def_delegators :fc_software_interface, :target_software
    def_delegator :fc_software_interface, :current_measurement_scale, :measurement_scale

    attr_reader :fc_measurements, :true_measurements, :previous_offset, :previous_scale
    attr_accessor :acquisition_time

    private

    def read_previous_offset_and_scale
        previous_offset_and_scale = fc_software_interface.current_offset_and_scale
        @previous_offset = previous_offset_and_scale.offset
        @previous_scale = previous_offset_and_scale.scale
        print "Previous current sensor configuration: offset = #{previous_offset.round(4)} V, scale = #{previous_scale.round(4)} A/V\n\n"
    end

    def gather_at_least_2_measurements
        gather_measurements
        raise TooFewMeasurements if fc_measurements.count < 2
    rescue TooFewMeasurements => error
        begin
            if Term.ask_yes_no "Not enough samples (acquired #{fc_measurements.count}, minimum 2), do you want to abort calibration (yn) [n] ? ", false
                raise
            else
                retry
            end
        rescue Interrupt
            puts
            raise error, caller[1..-1]
        end
    end

    def gather_measurements
        loop do
            Term.message_pause "Start the load and press enter to start a current measurement or press ctrl-c to stop the measurements..."

            begin
                fc_measure = acquire
                true_value = Term.ask_for_float 'Enter the real current in amperes > '
                puts

                fc_measurements << fc_measure
                true_measurements << true_value
            rescue AcquisitionAborted, Interrupt
                puts
            end

        end
    rescue Interrupt
        puts
    end

    def acquire
        samples = Array.new
        STDOUT.one_line_progress do
            sampling_start = Time.now
            loop do
                time_left = acquisition_time - (Time.now - sampling_start)
                break if time_left <= 0
                current = fc_software_interface.sample_current
                samples << current
                STDOUT.update_line "Time left: #{"%2.1f" % time_left}s - Samples: #{"%3d" % samples.count}"
            end
        end
        print "\a"
        samples.sum / samples.count
        #(samples.sum / samples.count).tap { |v| puts v.round(3) }
    rescue Interrupt
        STDERR.print "Acquisition aborted by user request\n\n"
        raise AcquisitionAborted
    end

    def calculate_offset_and_scale
        @offset_and_scale_calculator = CurrentScaleAndOffsetCalculator.get(target_software).new measurement_scale, fc_measurements, true_measurements
    end

    def check_results_sanity
        raise InvalidResults, "Something went wrong, the current calibration gave invalid results" if [ calc.offset, calc.scale ].any? { |v| v.nan? or v.infinite? }
    end

    def display_results
        meas_str = fc_measurements.map { |value| value.round 4 }
        true_str = true_measurements.map { |value| value.round 4 }
        coeffs_str = calc.coeffs.map { |coeff| coeff.round 4 }
        offsets_str = calc.offsets.map { |offset| offset.round 4 }

        puts
        puts "Offset: #{calc.offset.round 4} #{calc.offset_unit}"
        puts "Offset voltage: #{calc.voltage_offset.round 4} V#{" (#{target_software} offset value: #{calc.native_voltage_offset.round})" if target_software == :inav}"
        puts "Scale: #{calc.scale.round 4} #{calc.scale_unit}#{" (#{target_software} scale value: #{calc.native_scale.round})" if target_software == :inav}"
    end

    def save_calibration_onto_fc
        puts
        if Term.ask_yes_no 'Save new offset and scale onto FC (yn) ? [y] ', true
            fc_software_interface.set_current_offset_and_scale voltage_offset, scale
            fc_software_interface.save_settings
            puts "New current offset and scale saved onto FC"
        end
    end

    def restore_previous_offset_and_scale_onto_fc
        fc_software_interface.set_current_offset_and_scale previous_offset, previous_scale
    end

    attr_reader :fc_software_interface, :offset_and_scale_calculator
    alias calc offset_and_scale_calculator

end
