package radar

import "math"

// speedOfLight is the propagation constant used for axis conversion.
const speedOfLight = 299792458.0

// SpectrumConfig controls 2-D spectrum construction.
type SpectrumConfig struct {
	Window WindowConfig
}

// ComputeRangeDoppler runs the range FFT and Doppler FFT and returns
// the magnitude spectrum plus physical range/velocity axes.
func ComputeRangeDoppler(frame *Frame, cfg SpectrumConfig) (*RangeDoppler, error) {
	prepared, err := PreparePulses(frame, cfg.Window)
	if err != nil {
		return nil, err
	}
	nR := prepared.RangeFFTSize
	nD := prepared.DopplerSize

	// Fast-time (range) FFT per pulse.
	for p := 0; p < prepared.PulseCount; p++ {
		FFT(prepared.Re[p], prepared.Im[p])
	}

	// Slow-time DC removal after the range FFT: for each range bin subtract
	// the complex mean across pulses, which lands entirely in the zero-Doppler
	// bin. Removing it here (rather than in the time domain) guarantees that
	// static clutter/DC offset cannot produce a false target at v=0.
	if cfg.Window.RemoveDC {
		for r := 0; r < nR; r++ {
			var meanRe, meanIm float64
			for p := 0; p < prepared.PulseCount; p++ {
				meanRe += prepared.Re[p][r]
				meanIm += prepared.Im[p][r]
			}
			meanRe /= float64(prepared.PulseCount)
			meanIm /= float64(prepared.PulseCount)
			for p := 0; p < prepared.PulseCount; p++ {
				prepared.Re[p][r] -= meanRe
				prepared.Im[p][r] -= meanIm
			}
		}
	}

	mag := make([][]float64, nD)
	for d := 0; d < nD; d++ {
		mag[d] = make([]float64, nR)
	}
	reCol := make([]float64, nD)
	imCol := make([]float64, nD)
	for r := 0; r < nR; r++ {
		for p := 0; p < nD; p++ {
			if p < prepared.PulseCount {
				w := prepared.DopplerWin[p]
				reCol[p] = prepared.Re[p][r] * w
				imCol[p] = prepared.Im[p][r] * w
			} else {
				reCol[p] = 0
				imCol[p] = 0
			}
		}
		FFT(reCol, imCol)
		for d := 0; d < nD; d++ {
			mag[d][r] = math.Hypot(reCol[d], imCol[d])
		}
	}

	h := frame.Header
	rangeAxis := make([]float64, nR)
	binHz := h.SampleRateHz / float64(nR)
	for r := 0; r < nR; r++ {
		rangeAxis[r] = float64(r) * binHz * speedOfLight / (2 * h.ChirpSlope)
	}
	lambda := speedOfLight / h.CarrierFreqHz
	dopplerAxis := make([]float64, nD)
	unamb := float64(nD) * h.PRISec // sampling grid scale factor
	for d := 0; d < nD; d++ {
		v := float64(d) / unamb * lambda / 2
		if d > nD/2 {
			v -= lambda / (2 * h.PRISec)
		}
		dopplerAxis[d] = v
	}
	return &RangeDoppler{
		Magnitude:   mag,
		RangeAxis:   rangeAxis,
		DopplerAxis: dopplerAxis,
	}, nil
}
