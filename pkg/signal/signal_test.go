package signal

import (
	"math"
	"math/cmplx"
	"testing"
)

func approxEq(a, b, tol float64) bool {
	return math.Abs(a-b) <= tol
}

func TestFFTImpulse(t *testing.T) {
	n := 16
	a := make([]complex128, n)
	a[0] = 1
	fft(a)
	for _, v := range a {
		if !approxEq(real(v), 1, 1e-12) || !approxEq(imag(v), 0, 1e-12) {
			t.Fatalf("impulse FFT must be flat ones, got %v", a)
		}
	}
}

func TestFFTComplexExponentialBin(t *testing.T) {
	n := 32
	k := 5
	a := make([]complex128, n)
	for i := 0; i < n; i++ {
		a[i] = cmplx.Rect(1, 2*math.Pi*float64(k)*float64(i)/float64(n))
	}
	fft(a)
	for i := range a {
		want := 0.0
		if i == k {
			want = float64(n)
		}
		if !approxEq(cmplx.Abs(a[i]), want, 1e-9) {
			t.Fatalf("bin %d magnitude %v want %v", i, cmplx.Abs(a[i]), want)
		}
	}
}

func TestRangeDopplerShapeAndRepro(t *testing.T) {
	pulses, samples := 8, 16
	iq := makeIQ(pulses, samples, 3, 0)
	cfg := Config{Window: WindowHann, RemoveDC: true, RangeZeroPad: 2, DopplerZeroPad: 2}
	s1, err := RangeDoppler(iq, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if s1.RangeBins != 17 || s1.DopplerBins != 16 {
		t.Fatalf("shape = %d x %d", s1.RangeBins, s1.DopplerBins)
	}
	s2, _ := RangeDoppler(cloneIQ(iq), cfg)
	for r := range s1.Data {
		for d := range s1.Data[r] {
			if s1.Data[r][d] != s2.Data[r][d] {
				t.Fatalf("non-reproducible spectrum at %d,%d", r, d)
			}
		}
	}
}

func TestDCRemovalSuppressesZeroBin(t *testing.T) {
	pulses, samples := 4, 16
	iq := make([][]complex128, pulses)
	for p := range iq {
		iq[p] = make([]complex128, samples)
		for n := range iq[p] {
			iq[p][n] = complex(5.0, -3.0)
		}
	}
	withDC, err := RangeDoppler(iq, Config{Window: WindowNone})
	if err != nil {
		t.Fatal(err)
	}
	noDC, err := RangeDoppler(iq, Config{Window: WindowNone, RemoveDC: true})
	if err != nil {
		t.Fatal(err)
	}
	if cmplx.Abs(noDC.Data[0][withDC.DopplerBins/2]) > 1e-10 {
		t.Fatalf("DC not removed: %v", noDC.Data[0][noDC.DopplerBins/2])
	}
	if cmplx.Abs(withDC.Data[0][withDC.DopplerBins/2]) < 1 {
		t.Fatalf("DC unexpectedly suppressed without removal")
	}
}

func TestInvalidConfig(t *testing.T) {
	if _, err := RangeDoppler(nil, Config{}); err == nil {
		t.Fatal("nil input must error")
	}
	iq := [][]complex128{{1, 2}, {3}}
	if _, err := RangeDoppler(iq, Config{}); err == nil {
		t.Fatal("ragged input must error")
	}
	iq = [][]complex128{{1, 2}}
	if _, err := RangeDoppler(iq, Config{Window: "triangular"}); err == nil {
		t.Fatal("unknown window must error")
	}
	if _, err := RangeDoppler(iq, Config{RangeZeroPad: -1}); err == nil {
		t.Fatal("negative padding must error")
	}
}

func makeIQ(pulses, samples, rangeK, dopplerK int) [][]complex128 {
	iq := make([][]complex128, pulses)
	for p := 0; p < pulses; p++ {
		iq[p] = make([]complex128, samples)
		for n := 0; n < samples; n++ {
			ph := 2*math.Pi*float64(rangeK)*float64(n)/float64(samples) +
				2*math.Pi*float64(dopplerK)*float64(p)/float64(pulses)
			iq[p][n] = cmplx.Rect(1, ph)
		}
	}
	return iq
}

func cloneIQ(in [][]complex128) [][]complex128 {
	out := make([][]complex128, len(in))
	for p := range in {
		out[p] = append([]complex128(nil), in[p]...)
	}
	return out
}
