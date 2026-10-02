// Package detect runs noise-adaptive 2D CA-CFAR thresholding and clusters the
// adjacent threshold crossings into resolved target measurements.
package detect

import (
	"math"

	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
	"github.com/guoyyit/radar-target-detection-go/pkg/signal"
)

// Speed of light used for bin-to-physical-unit conversion.
const c0 = 299_792_458.0

// Peak is one clustered target measurement in physical units.
type Peak struct {
	RangeBin   int
	DopplerBin int
	Range      float64 // meters
	Velocity   float64 // m/s, positive = receding
	Amplitude  float64 // spectrum magnitude at the peak cell
	SNRDB      float64 // peak power / local CA-CFAR noise floor, dB
}

// CFARConfig configures the cell-averaging detector.
type CFARConfig struct {
	GuardRange   int
	GuardDoppler int
	TrainRange   int
	TrainDoppler int
	AlphaDB      float64 // threshold above the estimated noise floor, dB
	SkipRangeDC  bool    // mask range bin 0 (residual DC/leakage)
	SkipNearBins int     // always mask the first N range bins (TX leakage)
}

// ClusterConfig controls merging neighboring threshold crossings.
type ClusterConfig struct {
	MaxRangeGap   int
	MaxDopplerGap int
	MinCells      int
	Interpolate   bool // parabolic sub-bin refinement
}

// Detect returns a binary mask of cells exceeding the CA-CFAR threshold and
// the per-cell training-cell mean power (noise floor estimate). Cells with no
// usable training window get floor 0 and are never flagged. The detector scans
// row-major; when averaging, cells already flagged as targets are excluded, so
// a strong neighbor cannot inflate the estimate and suppress a weak neighbor.
func Detect(spec *signal.Spectrum, cfg CFARConfig) (mask [][]bool, floor [][]float64) {
	rb, db := spec.RangeBins, spec.DopplerBins
	power := powerGrid(spec)
	mask = make([][]bool, rb)
	floor = make([][]float64, rb)
	for r := 0; r < rb; r++ {
		mask[r] = make([]bool, db)
		floor[r] = make([]float64, db)
	}
	alpha := math.Pow(10, cfg.AlphaDB/10)

	for r := 0; r < rb; r++ {
		for d := 0; d < db; d++ {
			if (cfg.SkipRangeDC && r == 0) || r < cfg.SkipNearBins {
				continue
			}
			f, n := trainingFloor(power, mask, r, d, cfg, db)
			if n == 0 {
				continue
			}
			floor[r][d] = f
			if power[r][d] > f*alpha {
				mask[r][d] = true
			}
		}
	}
	return mask, floor
}

func powerGrid(spec *signal.Spectrum) [][]float64 {
	p := make([][]float64, spec.RangeBins)
	for r := 0; r < spec.RangeBins; r++ {
		p[r] = make([]float64, spec.DopplerBins)
		for d := 0; d < spec.DopplerBins; d++ {
			z := spec.Data[r][d]
			p[r][d] = real(z)*real(z) + imag(z)*imag(z)
		}
	}
	return p
}

func trainingFloor(power [][]float64, mask [][]bool, cr, cd int, cfg CFARConfig, db int) (float64, int) {
	rb := len(power)
	var sum float64
	count := 0
	for r := maxInt(0, cr-cfg.TrainRange); r <= minInt(rb-1, cr+cfg.TrainRange); r++ {
		distR := absInt(r - cr)
		for off := -cfg.TrainDoppler; off <= cfg.TrainDoppler; off++ {
			d := ((cd+off)%db + db) % db
			distD := absInt(off)
			if r == cr && off == 0 {
				continue
			}
			if distR <= cfg.GuardRange && distD <= cfg.GuardDoppler {
				continue
			}
			if mask[r][d] {
				continue
			}
			sum += power[r][d]
			count++
		}
	}
	if count == 0 {
		return 0, 0
	}
	return sum / float64(count), count
}

// Cluster groups mask cells via BFS. Two flagged cells connect when separated
// by at most MaxRangeGap/MaxDopplerGap bins (Doppler wraps at the Nyquist
// edge). Components smaller than MinCells are discarded.
func Cluster(spec *signal.Spectrum, mask [][]bool, floor [][]float64, hdr frame.Header, ccfg ClusterConfig) []Peak {
	rb, db := spec.RangeBins, spec.DopplerBins
	visited := make([][]bool, rb)
	for r := range visited {
		visited[r] = make([]bool, db)
	}

	var peaks []Peak
	for r := 0; r < rb; r++ {
		for d := 0; d < db; d++ {
			if !mask[r][d] || visited[r][d] {
				continue
			}
			cells := flood(spec, mask, visited, r, d, ccfg)
			if len(cells) < ccfg.MinCells {
				continue
			}
			peaks = append(peaks, toPeak(spec, cells, floor, hdr, ccfg))
		}
	}
	return peaks
}

type cell struct {
	r, d  int
	power float64
}

func flood(spec *signal.Spectrum, mask, visited [][]bool, sr, sd int, cfg ClusterConfig) []cell {
	rb, db := spec.RangeBins, spec.DopplerBins
	type pos struct{ r, d int }
	queue := []pos{{sr, sd}}
	visited[sr][sd] = true
	var out []cell

	drMax := 1 + cfg.MaxRangeGap
	ddMax := 1 + cfg.MaxDopplerGap

	for len(queue) > 0 {
		p := queue[0]
		queue = queue[1:]
		z := spec.Data[p.r][p.d]
		pw := real(z)*real(z) + imag(z)*imag(z)
		out = append(out, cell{p.r, p.d, pw})

		for dr := -drMax; dr <= drMax; dr++ {
			nr := p.r + dr
			if nr < 0 || nr >= rb {
				continue
			}
			for dd := -ddMax; dd <= ddMax; dd++ {
				nd := ((p.d+dd)%db + db) % db
				if nr == p.r && nd == p.d {
					continue
				}
				if mask[nr][nd] && !visited[nr][nd] {
					visited[nr][nd] = true
					queue = append(queue, pos{nr, nd})
				}
			}
		}
	}
	return out
}

func toPeak(spec *signal.Spectrum, cells []cell, floor [][]float64, hdr frame.Header, cfg ClusterConfig) Peak {
	var wd, wp float64
	var pr, pd int
	best := -1.0
	for _, c := range cells {
		wd += float64(c.d) * c.power
		wp += c.power
		if c.power > best {
			best = c.power
			pr, pd = c.r, c.d
		}
	}

	rBin := float64(pr)
	dBin := wd / wp
	if cfg.Interpolate {
		if dr, ok := parabolicOffset(spec, pr, pd, true); ok {
			rBin = float64(pr) + dr
		}
		if dd, ok := parabolicOffset(spec, pr, pd, false); ok {
			dBin = wrapDoppler(float64(pd)+dd, spec.DopplerBins)
		}
	}

	z := spec.Data[pr][pd]
	amp := signal.Magnitude(z)
	fftRangeN := 2 * (spec.RangeBins - 1)
	snr := 0.0
	if floor != nil && floor[pr][pd] > 0 {
		pk := real(z)*real(z) + imag(z)*imag(z)
		snr = 10 * math.Log10(pk/floor[pr][pd])
	}

	return Peak{
		RangeBin:   pr,
		DopplerBin: pd,
		Range:      binToRange(rBin, hdr, fftRangeN),
		Velocity:   binToVelocity(dBin, hdr, spec.DopplerBins),
		Amplitude:  amp,
		SNRDB:      snr,
	}
}

func wrapDoppler(v float64, n int) float64 {
	f := float64(n)
	for v < 0 {
		v += f
	}
	for v >= f {
		v -= f
	}
	return v
}

// parabolicOffset fits a parabola to the peak cell and its two neighbors and
// returns the fractional offset of the fitted maximum.
func parabolicOffset(spec *signal.Spectrum, r, d int, alongRange bool) (float64, bool) {
	rb, db := spec.RangeBins, spec.DopplerBins
	y1 := powerAt(spec, r, d)
	var y0, y2 float64
	if alongRange {
		if r-1 < 0 || r+1 >= rb {
			return 0, false
		}
		y0 = powerAt(spec, r-1, d)
		y2 = powerAt(spec, r+1, d)
	} else {
		y0 = powerAt(spec, r, (d-1+db)%db)
		y2 = powerAt(spec, r, (d+1)%db)
	}
	den := y0 - 2*y1 + y2
	if den >= 0 {
		return 0, false
	}
	return 0.5 * (y0 - y2) / den, true
}

func powerAt(spec *signal.Spectrum, r, d int) float64 {
	z := spec.Data[r][d]
	return real(z)*real(z) + imag(z)*imag(z)
}

func binToRange(bin float64, hdr frame.Header, fftN int) float64 {
	freq := bin * hdr.SampleRate / float64(fftN)
	return freq * c0 / (2 * hdr.ChirpSlope)
}

func binToVelocity(bin float64, hdr frame.Header, dopplerBins int) float64 {
	// Bins are fftshifted: move zero-frequency back to index 0.
	freq := (bin - float64(dopplerBins/2)) / float64(dopplerBins) / hdr.PulseInterval
	return freq * c0 / (2 * hdr.CarrierFreq)
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
