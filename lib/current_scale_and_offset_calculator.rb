
module NumericArray

    def round decimals
        map { |v| (v.is_a?(Float) or v.is_a?(Rational)) ? v.to_f.round(decimals) : v }
    end

end

class CurrentScaleAndOffsetCalculator

    def self.get software
        case software
        when :ardupilot then ArdupilotCurrentScaleAndOffsetCalculator
        when :inav then INavCurrentScaleAndOffsetCalculator
        else raise ArgumentError, "unsupported software: #{software}"
        end
    end

    def initialize orig_scale, meas, real
        raise ArgumentError, "Not the same number of meas (#{meas.count}) and real (#{real.count}) values" if meas.count != real.count
        @orig_scale = orig_scale
        @meas = meas.extend NumericArray
        @real = real.extend NumericArray
    end

    def coeffs
        @coeffs ||= meas.zip(real).each_cons(2).map { |(meas0, real0), (meas1, real1)| (meas1 - meas0) / (real1 - real0) }.extend NumericArray
    end

    def coeff_avg
        coeffs.sum / coeffs.count
    end

    alias coeff coeff_avg

    def scale_adjusted_meas
        @scale_adjusted_meas ||= meas.map { |mv| mv / coeff_avg }.extend NumericArray
    end

    def offsets
        @current_offsets ||= scale_adjusted_meas.zip(real).map { |nmv, rv| nmv - rv }.extend NumericArray
    end

    def avg_offset
        offsets.sum / offsets.count
    end

    alias offset avg_offset

    def offset_unit
        "A"
    end

    attr_reader :orig_scale, :meas, :real

end

class INavCurrentScaleAndOffsetCalculator < CurrentScaleAndOffsetCalculator

    def initialize orig_scale, meas, real
        super orig_scale, meas, real
    end

    def voltage_offset
        offset * scale
    end

    def scale
        orig_scale * coeff_avg
    end

    def scale_unit
        "V/A"
    end

    def native_scale
        (scale * 10000).round
    end

    def native_scale_unit
        "0.1mV/A"
    end

    def native_voltage_offset
        (voltage_offset * 10000).round
    end

    def native_voltage_offset_unit
        "0.1mV"
    end

end

class ArdupilotCurrentScaleAndOffsetCalculator < CurrentScaleAndOffsetCalculator

    def scale
        orig_scale / coeff_avg
    end

    def scale_unit
        "A/V"
    end

    def voltage_offset
        offset / scale
    end

    def voltage_offset_unit
        "0.1mV"
    end

    alias native_scale scale
    alias native_scale_unit scale_unit
    alias native_voltage_offset voltage_offset
    alias native_voltage_offset_unit voltage_offset_unit

end

