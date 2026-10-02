package radar

import "math"

// CFARConfig configures 2-D cell-averaging CFAR detection.
type CFARConfig struct {
	GuardRange   int
	GuardDoppler int
	TrainRange   int
	TrainDoppler int
	Alpha        float64 // threshold multiplier applied to the noise estimate
	ThresholdDB  float64 // minimum absolute SNR in dB
	SkipDCBins   int     // range bins around index 0 excluded after DC removal
	// SearchRangeBins limits the searched range bins; 0 means 0..Nyquist.
	SearchRangeBins int
	// MinTrainingCells rejects borders with too few training samples; 0 => 4.
	MinTrainingCells int
}

func halfRangeBins(nR int) int {
	return nR/2 + 1
}

// Detect returns per-cell detections above the adaptive threshold.
func Detect(rd *RangeDoppler, cfg CFARConfig) []Detection {
	nD := len(rd.Magnitude)
	if nD == 0 {
		return nil
	}
	nR := len(rd.Magnitude[0])
	searchR := cfg.SearchRangeBins
	if searchR <= 0 || searchR > nR {
		searchR = halfRangeBins(nR)
	}
	minTrain := cfg.MinTrainingCells
	if minTrain <= 0 {
		minTrain = 4
	}
	var out []Detection
	for d := 0; d < nD; d++ {
		for r := cfg.SkipDCBins; r < searchR; r++ {
			mag := rd.Magnitude[d][r]
			power := mag * mag
			noisePower, trainCount := cfarNoise(rd, d, r, nD, nR, cfg)
			if trainCount < minTrain || noisePower <= 0 {
				continue
			}
			if power < cfg.Alpha*noisePower {
				continue
			}
			snr := 10 * math.Log10(power/noisePower)
			if snr < cfg.ThresholdDB {
				continue
			}
			if !isLocalPeak(rd, d, r, nD, searchR) {
				continue
			}
			out = append(out, Detection{
				RangeBin:   r,
				DopplerBin: d,
				Range:      rd.RangeAxis[r],
				Velocity:   rd.DopplerAxis[d],
				Amplitude:  mag,
				Noise:      math.Sqrt(noisePower),
				SNRDB:      snr,
			})
		}
	}
	return out
}

func cfarNoise(rd *RangeDoppler, d, r, nD, nR int, cfg CFARConfig) (float64, int) {
	sum := 0.0
	count := 0
	spanD := cfg.GuardDoppler + cfg.TrainDoppler
	spanR := cfg.GuardRange + cfg.TrainRange
	for dd := d - spanD; dd <= d+spanD; dd++ {
		if dd < 0 || dd >= nD {
			continue
		}
		for rr := r - spanR; rr <= r+spanR; rr++ {
			if rr < 0 || rr >= nR {
				continue
			}
			if absInt(dd-d) <= cfg.GuardDoppler && absInt(rr-r) <= cfg.GuardRange {
				continue
			}
			m := rd.Magnitude[dd][rr]
			sum += m * m
			count++
		}
	}
	if count == 0 {
		return 0, 0
	}
	return sum / float64(count), count
}

func isLocalPeak(rd *RangeDoppler, d, r, nD, searchR int) bool {
	v := rd.Magnitude[d][r]
	for dd := d - 1; dd <= d+1; dd++ {
		ddw := dd
		if ddw < 0 {
			ddw += nD
		} else if ddw >= nD {
			ddw -= nD
		}
		for rr := r - 1; rr <= r+1; rr++ {
			if rr < 0 || rr >= searchR {
				continue
			}
			if ddw == d && rr == r {
				continue
			}
			if rd.Magnitude[ddw][rr] > v {
				return false
			}
		}
	}
	return true
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
