package radar

import (
	"errors"
	"fmt"
	"math"
)

// WindowKind selects the per-axis window.
type WindowKind string

const (
	WindowRect     WindowKind = "rect"
	WindowHann     WindowKind = "hann"
	WindowHamming  WindowKind = "hamming"
	WindowBlackman WindowKind = "blackman"
)

// WindowConfig configures DC removal, windowing and zero padding.
type WindowConfig struct {
	RangeWindow   WindowKind
	DopplerWindow WindowKind
	RemoveDC      bool
	// RangePadTo/DopplerPadTo are FFT sizes; 0 means no padding. Values are
	// rounded up to a power of two and must be >= the input length.
	RangePadTo   int
	DopplerPadTo int
}

// PreparedPulses holds windowed/offset/padded complex sample matrices.
type PreparedPulses struct {
	Re           [][]float64 // [pulse][sample]
	Im           [][]float64
	DopplerWin   []float64
	PulseCount   int
	SampleCount  int
	RangeFFTSize int
	DopplerSize  int
}

// PreparePulses removes DC, applies windows and zero-pads the frame.
func PreparePulses(frame *Frame, cfg WindowConfig) (*PreparedPulses, error) {
	nP := len(frame.Pulses)
	nS := frame.Header.SamplesPerPulse
	if nP == 0 || nS == 0 {
		return nil, errors.New("empty frame")
	}
	rangeFFT := cfg.RangePadTo
	if rangeFFT == 0 {
		rangeFFT = nS
	}
	rangeFFT = nextPow2(rangeFFT)
	if rangeFFT < nS {
		return nil, fmt.Errorf("range FFT size %d smaller than samples %d", rangeFFT, nS)
	}
	dopFFT := cfg.DopplerPadTo
	if dopFFT == 0 {
		dopFFT = nP
	}
	dopFFT = nextPow2(dopFFT)
	if dopFFT < nP {
		return nil, fmt.Errorf("doppler FFT size %d smaller than pulses %d", dopFFT, nP)
	}
	winR, err := makeWindow(cfg.RangeWindow, nS)
	if err != nil {
		return nil, err
	}
	winD, err := makeWindow(cfg.DopplerWindow, nP)
	if err != nil {
		return nil, err
	}
	re := make([][]float64, nP)
	im := make([][]float64, nP)
	for p := 0; p < nP; p++ {
		re[p] = make([]float64, rangeFFT)
		im[p] = make([]float64, rangeFFT)
	}
	// Copy samples, remove per-pulse (fast-time) DC, and apply range window.
	for p, pulse := range frame.Pulses {
		var meanI, meanQ float64
		if cfg.RemoveDC {
			for k := 0; k < nS; k++ {
				meanI += pulse.I[k]
				meanQ += pulse.Q[k]
			}
			meanI /= float64(nS)
			meanQ /= float64(nS)
		}
		for k := 0; k < nS; k++ {
			re[p][k] = (pulse.I[k] - meanI) * winR[k]
			im[p][k] = (pulse.Q[k] - meanQ) * winR[k]
		}
	}
	return &PreparedPulses{
		Re:           re,
		Im:           im,
		DopplerWin:   winD,
		PulseCount:   nP,
		SampleCount:  nS,
		RangeFFTSize: rangeFFT,
		DopplerSize:  dopFFT,
	}, nil
}

func makeWindow(kind WindowKind, n int) ([]float64, error) {
	w := make([]float64, n)
	switch kind {
	case "", WindowRect:
		for i := range w {
			w[i] = 1
		}
	case WindowHann:
		for i := 0; i < n; i++ {
			w[i] = 0.5 - 0.5*cos2pi(i, n-1)
		}
	case WindowHamming:
		for i := 0; i < n; i++ {
			w[i] = 0.54 - 0.46*cos2pi(i, n-1)
		}
	case WindowBlackman:
		for i := 0; i < n; i++ {
			c1 := cos2pi(i, n-1)
			c2 := cos2pi(2*i, n-1)
			w[i] = 0.42 - 0.5*c1 + 0.08*c2
		}
	default:
		return nil, fmt.Errorf("unknown window kind %q", kind)
	}
	return w, nil
}

func cos2pi(i, denom int) float64 {
	if denom == 0 {
		return 1
	}
	return math.Cos(2 * math.Pi * float64(i) / float64(denom))
}
