package radar

import "math"

// FFT performs an in-place complex DFT. When n is a power of two a radix-2
// decimation-in-time Cooley-Tukey transform is used; otherwise a naive DFT is
// computed. re[k], im[k] hold input on entry and output on return. The
// computation is deterministic for identical inputs.
func FFT(re, im []float64) {
	n := len(re)
	if n != len(im) || n < 2 {
		return
	}
	if n&(n-1) == 0 {
		fftPow2(re, im)
		return
	}
	outRe := make([]float64, n)
	outIm := make([]float64, n)
	for k := 0; k < n; k++ {
		var sumRe, sumIm float64
		angle := -2 * math.Pi * float64(k) / float64(n)
		for t := 0; t < n; t++ {
			phi := angle * float64(t)
			c, s := math.Cos(phi), math.Sin(phi)
			sumRe += re[t]*c - im[t]*s
			sumIm += re[t]*s + im[t]*c
		}
		outRe[k] = sumRe
		outIm[k] = sumIm
	}
	copy(re, outRe)
	copy(im, outIm)
}

func fftPow2(re, im []float64) {
	n := len(re)
	for i, j := 1, 0; i < n; i++ {
		bit := n >> 1
		for j&bit != 0 {
			j ^= bit
			bit >>= 1
		}
		j ^= bit
		if i < j {
			re[i], re[j] = re[j], re[i]
			im[i], im[j] = im[j], im[i]
		}
	}
	for length := 2; length <= n; length <<= 1 {
		half := length >> 1
		step := -2 * math.Pi / float64(length)
		for start := 0; start < n; start += length {
			for k := 0; k < half; k++ {
				phi := step * float64(k)
				c, s := math.Cos(phi), math.Sin(phi)
				a := start + k
				b := a + half
				tRe := c*re[b] - s*im[b]
				tIm := c*im[b] + s*re[b]
				re[b] = re[a] - tRe
				im[b] = im[a] - tIm
				re[a] += tRe
				im[a] += tIm
			}
		}
	}
}

// nextPow2 returns the smallest power of two >= n (n >= 1).
func nextPow2(n int) int {
	p := 1
	for p < n {
		p <<= 1
	}
	return p
}
