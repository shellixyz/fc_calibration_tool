class CalibrationSampleStats

    DEFAULT_QUALITY_THRESHOLDS = {
        excellent: 90.0,
        good: 75.0,
        fair: 50.0
    }.freeze

    def self.from_samples samples
        raise ArgumentError, 'samples must not be empty' if samples.empty?

        count = samples.count
        mean = samples.sum / count.to_f
        variance = samples.sum { |sample| (sample - mean) ** 2 } / count.to_f

        new count, mean, Math.sqrt(variance), samples.min, samples.max
    end

    def initialize count, mean, standard_deviation, minimum, maximum
        @count = count
        @mean = mean
        @standard_deviation = standard_deviation
        @minimum = minimum
        @maximum = maximum
    end

    def span
        maximum - minimum
    end

    def relative_noise
        return 0.0 if mean.zero? && standard_deviation.zero?

        standard_deviation / [ mean.abs, 1e-9 ].max
    end

    def quality_score
        raw_score = 100.0 * (1.0 - relative_noise * 5.0)
        [ [ raw_score, 0.0 ].max, 100.0 ].min
    end

    def quality_label thresholds = DEFAULT_QUALITY_THRESHOLDS
        score = quality_score

        case score
        when thresholds[:excellent].. then 'excellent'
        when thresholds[:good]...thresholds[:excellent] then 'good'
        when thresholds[:fair]...thresholds[:good] then 'fair'
        else 'poor'
        end
    end

    attr_reader :count, :mean, :standard_deviation, :minimum, :maximum

end