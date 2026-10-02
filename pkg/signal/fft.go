package signal

import "math"

// fft performs an in-place iterative radix-2 decimation-in-time FFT. len(a)
// must be a power of two (guaranteed by RangeDoppler's padding). It uses no
// concurrency so bit-for-bit identical inputs always produce identical output.
func fft(a []complex128) {
	n := len(a)
	if n <= 1 {
		return
	}

	// Bit-reversal permutation.
	for i, j := 1, 0; i < n; i++ {
		bit := n >> 1
		for j&bit != 0 {
			j ^= bit
			bit >>= 1
		}
		j ^= bit
		if i < j {
			a[i], a[j] = a[j], a[i]
		}
	}

	for length := 2; length <= n; length <<= 1 {
		half := length >> 1
		angle := -2 * math.Pi / float64(length)
		rootStep := complex(math.Cos(angle), math.Sin(angle))
		for start := 0; start < n; start += length {
			twiddle := 1 + 0i
			for k := 0; k < half; k++ {
				u := a[start+k]
				v := a[start+k+half] * twiddle
				a[start+k] = u + v
				a[start+k+half] = u - v
				twiddle *= rootStep
			}
		}
	}
}
