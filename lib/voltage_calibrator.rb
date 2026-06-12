
require 'forwardable'
require_relative 'calibration_sample_stats'
require_relative 'libutil/lib/term'

class VoltageCalibrator

    DEFAULT_ACQUISITION_TIME = 5 # seconds

    def initialize fc_software_interface, acquisition_time: DEFAULT_ACQUISITION_TIME, quality_thresholds: CalibrationSampleStats::DEFAULT_QUALITY_THRESHOLDS
        @acquisition_time = acquisition_time
        @measurement_stats = nil
        @fc_software_interface = fc_software_interface
        @quality_thresholds = quality_thresholds
    end

    def run
        check_sensor_enabled
        puts '*' * 20 + ' VOLTAGE CALIBRATION ' + '*' * 20 + "\n\n"
        read_previous_scale
        fc_software_interface.prepare_voltage_calibration
        gather_measurement
        calculate_scale
        display_results
        save_calibration_onto_fc
    rescue Interrupt
        restore_previous_scale_onto_fc
        STDERR.puts "Voltage calibration aborted"
    end

    def check_sensor_enabled
        raise "Voltage sensor not enabled" unless fc_software_interface.voltage_sensor_enabled?
    end

    def calibration_data
        raise 'no data available' if scale.nil?
        {
            voltage_scale: scale,
            measurement_stats: measurement_stats && {
                count: measurement_stats.count,
                mean: measurement_stats.mean,
                standard_deviation: measurement_stats.standard_deviation,
                minimum: measurement_stats.minimum,
                maximum: measurement_stats.maximum,
                span: measurement_stats.span,
                quality_score: measurement_stats.quality_score,
                quality_label: measurement_stats.quality_label(quality_thresholds)
            }
        }
    end

    extend Forwardable
    def_delegator :fc_software_interface, :voltage_measurement_scale, :measurement_scale
    def_delegators :fc_software_interface, :target_software

    attr_reader :previous_scale, :fc_measure, :true_value, :scale, :measurement_stats
    attr_accessor :acquisition_time

    private

    def read_previous_scale
        @previous_scale = fc_software_interface.voltage_scale
        print "Previous voltage sensor scale = #{previous_scale.round(4)}\n\n"
    end

    def gather_measurement
        Term.message_pause "Press enter to start the voltage measurement or press ctrl-c to abort the voltage calibration..."
        @fc_measure, @measurement_stats = acquire
        display_measurement_quality measurement_stats, 'V'
        @true_value = Term.ask_for_float 'Enter the real voltage in volts > '
        puts
    rescue Interrupt
        puts
        raise
    end

    def acquire
        samples = Array.new
        STDOUT.one_line_progress do
            sampling_start = Time.now
            loop do
                time_left = acquisition_time - (Time.now - sampling_start)
                break if time_left <= 0
                voltage = fc_software_interface.sample_voltage
                samples << voltage
                STDOUT.update_line "Time left: #{"%2.1f" % time_left}s - Samples: #{"%3d" % samples.count}"
            end
        end
        print "\a"
        stats = CalibrationSampleStats.from_samples samples
        [ stats.mean, stats ]
    end

    def display_measurement_quality stats, unit
        puts "Measurement quality: #{stats.quality_score.round}/100 (#{stats.quality_label(quality_thresholds)}) - avg=#{stats.mean.round(4)} #{unit}, min=#{stats.minimum.round(4)} #{unit}, max=#{stats.maximum.round(4)} #{unit}, span=#{stats.span.round(4)} #{unit}, sd=#{stats.standard_deviation.round(4)} #{unit}, samples=#{stats.count}"
    end

    def calculate_scale
        @scale = measurement_scale * true_value / fc_measure
    end

    def display_results
        puts "Adjusted voltage scale: #{scale.round(4)}"
        puts "Measurement quality: #{measurement_stats.quality_score.round}/100" if measurement_stats
    end

    def save_calibration_onto_fc
        puts
        if Term.ask_yes_no 'Save new offset and scale onto FC (yn) ? [y] ', true
            fc_software_interface.set_voltage_scale scale
            fc_software_interface.save_settings
            puts "New voltage scale saved onto FC"
        end
    end

    def restore_previous_scale_onto_fc
        fc_software_interface.set_voltage_scale previous_scale
    end

    attr_reader :fc_software_interface, :fc_measure, :true_value

    attr_reader :quality_thresholds

end
