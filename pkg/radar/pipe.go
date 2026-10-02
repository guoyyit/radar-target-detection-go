package radar

// Pipe is an in-memory bounded channel simulating the local echo input.
type Pipe struct {
	frames chan RawFrame
	done   chan struct{}
}

// NewPipe creates an in-memory pipe with the given buffer size.
func NewPipe(buffer int) *Pipe {
	if buffer < 0 {
		buffer = 0
	}
	return &Pipe{
		frames: make(chan RawFrame, buffer),
		done:   make(chan struct{}),
	}
}

// Push enqueues a frame, blocking while the buffer is full. It returns false
// if the pipe was closed.
func (p *Pipe) Push(f RawFrame) bool {
	select {
	case p.frames <- f:
		return true
	case <-p.done:
		return false
	}
}

// Close signals end of input; queued frames remain readable.
func (p *Pipe) Close() {
	select {
	case <-p.done:
	default:
		close(p.done)
	}
}

// Recv dequeues a frame. ok is false only after Close and the queue is empty.
func (p *Pipe) Recv() (f RawFrame, ok bool) {
	select {
	case f, ok = <-p.frames:
		return f, ok
	default:
	}
	select {
	case f, ok = <-p.frames:
		return f, ok
	case <-p.done:
		select {
		case f, ok = <-p.frames:
			return f, ok
		default:
			return RawFrame{}, false
		}
	}
}
