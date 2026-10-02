// Package signal performs preprocessing (windowing, DC removal, zero padding)
// and the two-dimensional range-Doppler spectrum transform.
package signal

import (
	"errors"
	"math"
)

// Window selects the time-domain tapering window.
type Window string

const (
	WindowNone     Window = "none"
	WindowHann     Window = "hann"
	WindowHamming  Window = "hamming"
	WindowBlackman Window = "blackman"
)

// Config controls preprocessing and FFT sizing. Zero padding factors of 0 or
// 1 disable padding for that axis.
type Config struct {
	Window         Window
	RemoveDC       bool
	RangeZeroPad   int
	DopplerZeroPad int
}

// ErrInvalidConfig reports an unsupported window or negative padding factor.
var ErrInvalidConfig = errors.New("signal: invalid configuration")

// Spectrum is a range-Doppler complex spectrum indexed [rangeBin][dopplerBin].
// Doppler bins are fftshifted so zero velocity is the center column.
type Spectrum struct {
	RangeBins   int
	DopplerBins int
	Data        [][]complex128
}

// RangeDoppler computes the 2D spectrum of IQ[pulse][sample].
//
// Processing per pulse: optional mean subtraction (DC removal), windowing,
// zero padding, fast-time FFT. Then across pulses for every range bin:
// windowing in slow time, zero padding, FFT, and fftshift on the Doppler
// axis. Only positive (one-sided) range bins are returned.
func RangeDoppler(iq [][]complex128, cfg Config) (*Spectrum, error) {
	if len(iq) == 0 || len(iq[0]) == 0 {
		return nil, ErrInvalidConfig
	}
	pulses := len(iq)
	samples := len(iq[0])
	for p := 0; p < pulses; p++ {
		if len(iq[p]) != samples {
			return nil, ErrInvalidConfig
		}
	}
	rwin, err := window(cfg.Window, samples)
	if err != nil {
		return nil, err
	}
	dwin, err := window(cfg.Window, pulses)
	if err != nil {
		return nil, err
	}
	if cfg.RangeZeroPad < 0 || cfg.DopplerZeroPad < 0 {
		return nil, ErrInvalidConfig
	}

	rangeN := nextPow2(samples * maxInt(1, cfg.RangeZeroPad))
	dopplerN := nextPow2(pulses * maxInt(1, cfg.DopplerZeroPad))
	rangeBins := rangeN/2 + 1

	// rangeFFT[p] holds the one-sided fast-time spectrum of pulse p.
	rangeFFT := make([][]complex128, pulses)
	buf := make([]complex128, rangeN)
	for p := 0; p < pulses; p++ {
		for s := 0; s < rangeN; s++ {
			buf[s] = 0
		}
		if cfg.RemoveDC {
			var meanI, meanQ float64
			for s := 0; s < samples; s++ {
				meanI += real(iq[p][s])
				meanQ += imag(iq[p][s])
			}
			meanI /= float64(samples)
			meanQ /= float64(samples)
			for s := 0; s < samples; s++ {
				buf[s] = complex(real(iq[p][s])-meanI, imag(iq[p][s])-meanQ) * rwin[s]
			}
		} else {
			for s := 0; s < samples; s++ {
				buf[s] = iq[p][s] * rwin[s]
			}
		}
		fft(buf)
		rangeFFT[p] = make([]complex128, rangeBins)
		copy(rangeFFT[p], buf[:rangeBins])
	}

	// Slow-time FFT for each range bin, with fftshift output.
	data := make([][]complex128, rangeBins)
	slow := make([]complex128, dopplerN)
	for rb := 0; rb < rangeBins; rb++ {
		for k := range slow {
			slow[k] = 0
		}
		for p := 0; p < pulses; p++ {
			slow[p] = rangeFFT[p][rb] * dwin[p]
		}
		fft(slow)
		data[rb] = make([]complex128, dopplerN)
		copy(data[rb], slow)
		fftShift(data[rb])
	}

	return &Spectrum{
		RangeBins:   rangeBins,
		DopplerBins: dopplerN,
		Data:        data,
	}, nil
}

// Magnitude returns |z| for one cell.
func Magnitude(z complex128) float64 {
	// math.Hypot guards against overflow at high FFT gain.
	return math.Hypot(real(z), imag(z))
}

func window(kind Window, n int) ([]complex128, error) {
	w := make([]complex128, n)
	switch kind {
	case "", WindowNone:
		for i := 0; i < n; i++ {
			w[i] = 1
		}
	case WindowHann:
		for i := 0; i < n; i++ {
			w[i] = complex(0.5-0.5*cos2pi(float64(i)/float64(n-1)), 0)
		}
	case WindowHamming:
		for i := 0; i < n; i++ {
			w[i] = complex(0.54-0.46*cos2pi(float64(i)/float64(n-1)), 0)
		}
	case WindowBlackman:
		for i := 0; i < n; i++ {
			t := float64(i) / float64(n-1)
			w[i] = complex(0.42-0.5*cos2pi(t)+0.08*cos4pi(t), 0)
		}
	default:
		return nil, ErrInvalidConfig
	}
	return w, nil
}

func cos2pi(t float64) float64 {
	return math.Cos(2 * math.Pi * t)
}

func cos4pi(t float64) float64 {
	return math.Cos(4 * math.Pi * t)
}

func nextPow2(n int) int {
	p := 1
	for p < n {
		p <<= 1
	}
	return p
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func fftShift(a []complex128) {
	n := len(a)
	half := n / 2
	tmp := make([]complex128, n)
	copy(tmp, a[half:])
	copy(tmp[n-half:], a[:half])
	copy(a, tmp)
}
