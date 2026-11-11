"""Minimal TensorRT 10 runtime for the Jetson, with no CUDA Python packages.

TensorRT's own Python bindings are installed system-wide, but pycuda and
cuda-python are not, and installing them next to ROS's numpy is what the
project rules forbid. All TensorRT needs from CUDA is device memory, a stream
and copies, so those come straight from libcudart through ctypes.

Memory model (Jetson: CPU and GPU share DRAM):
  * every input and output has a PINNED host buffer exposed as a numpy array
    and a device buffer; copies are cudaMemcpyAsync on one stream;
  * the caller fills `inputs[name]` in place, calls `infer()`, and reads
    `outputs[name]`. No allocation happens per call.

Dynamic shapes (e.g. a re-ID batch) are supported: buffers are sized for the
profile's MAX shape once, and `infer(shapes={name: shape})` sets the actual
shape; the returned arrays are views trimmed to it.
"""

import ctypes
import ctypes.util

import numpy as np
import tensorrt as trt

_CUDA_MEMCPY_H2D = 1
_CUDA_MEMCPY_D2H = 2


def _load_cudart():
    for name in ('libcudart.so', 'libcudart.so.12',
                 '/usr/local/cuda/lib64/libcudart.so'):
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    path = ctypes.util.find_library('cudart')
    if path:
        return ctypes.CDLL(path)
    raise OSError('libcudart not found (CUDA runtime)')


_cudart = _load_cudart()
_cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
_cudart.cudaFree.argtypes = [ctypes.c_void_p]
_cudart.cudaHostAlloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t,
                                  ctypes.c_uint]
_cudart.cudaFreeHost.argtypes = [ctypes.c_void_p]
_cudart.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                    ctypes.c_int, ctypes.c_void_p]
_cudart.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
_cudart.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
_cudart.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
_cudart.cudaGetErrorString.restype = ctypes.c_char_p
_cudart.cudaGetErrorString.argtypes = [ctypes.c_int]


# Block (sleep) in cudaStreamSynchronize instead of spinning. CUDA's default
# on this Jetson spins a CPU core for the whole inference (~20 ms a frame);
# with the nav stack using 4-7 of 6 cores that is CPU stolen for nothing.
# Must be set before the first CUDA call creates the context; if a context
# already exists (another library got there first) it fails harmlessly.
_CUDA_DEVICE_SCHEDULE_BLOCKING_SYNC = 0x04
_cudart.cudaSetDeviceFlags.argtypes = [ctypes.c_uint]
BLOCKING_SYNC = _cudart.cudaSetDeviceFlags(_CUDA_DEVICE_SCHEDULE_BLOCKING_SYNC) == 0


def _check(err, what):
    if err != 0:
        msg = _cudart.cudaGetErrorString(err).decode()
        raise RuntimeError(f'{what} failed: CUDA error {err} ({msg})')


class _Buffer:
    """A pinned host array and a device allocation of the same byte size."""

    def __init__(self, shape, dtype):
        self.dtype = np.dtype(dtype)
        self.max_shape = tuple(int(d) for d in shape)
        self.nbytes = int(np.prod(self.max_shape)) * self.dtype.itemsize
        self.host_ptr = ctypes.c_void_p()
        self.dev_ptr = ctypes.c_void_p()
        _check(_cudart.cudaHostAlloc(ctypes.byref(self.host_ptr), self.nbytes, 0),
               'cudaHostAlloc')
        _check(_cudart.cudaMalloc(ctypes.byref(self.dev_ptr), self.nbytes), 'cudaMalloc')
        raw = (ctypes.c_uint8 * self.nbytes).from_address(self.host_ptr.value)
        self.host = np.frombuffer(raw, dtype=self.dtype).reshape(self.max_shape)

    def free(self):
        if self.dev_ptr.value:
            _cudart.cudaFree(self.dev_ptr)
            self.dev_ptr = ctypes.c_void_p()
        if self.host_ptr.value:
            self.host = None
            _cudart.cudaFreeHost(self.host_ptr)
            self.host_ptr = ctypes.c_void_p()


class TrtEngine:
    """Load a serialized TensorRT engine and run it synchronously."""

    def __init__(self, engine_path, max_shapes=None):
        """max_shapes: {tensor_name: shape} for dynamic inputs; defaults to
        the optimisation profile's maximum."""
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        with open(engine_path, 'rb') as f:
            self.engine = self._runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'could not deserialize {engine_path} '
                               '(built with a different TensorRT version?)')
        self.context = self.engine.create_execution_context()
        self._stream = ctypes.c_void_p()
        _check(_cudart.cudaStreamCreate(ctypes.byref(self._stream)), 'cudaStreamCreate')

        max_shapes = dict(max_shapes or {})
        self.input_names, self.output_names = [], []
        self._buffers = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
                shape = tuple(self.engine.get_tensor_shape(name))
                if any(d < 0 for d in shape):
                    shape = tuple(max_shapes.get(
                        name, self.engine.get_tensor_profile_shape(name, 0)[2]))
                self.context.set_input_shape(name, shape)
                self._buffers[name] = _Buffer(shape, dtype)
            else:
                self.output_names.append(name)
        # Output sizes follow from the (maximum) input shapes just set.
        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            if any(d < 0 for d in shape):
                raise RuntimeError(f'output {name} has unresolved shape {shape}')
            self._buffers[name] = _Buffer(shape, trt.nptype(self.engine.get_tensor_dtype(name)))
        for name, buf in self._buffers.items():
            self.context.set_tensor_address(name, buf.dev_ptr.value)

        self.inputs = {n: self._buffers[n].host for n in self.input_names}
        self.outputs = {n: self._buffers[n].host for n in self.output_names}

    def input_shape(self, name):
        return self._buffers[name].max_shape

    def infer(self, shapes=None):
        """Run once. shapes: {input_name: actual_shape} for dynamic inputs
        (the data must already be in the leading part of inputs[name]).
        Returns {output_name: array view of the actual output shape}."""
        actual_in = {}
        for name in self.input_names:
            shape = tuple(shapes[name]) if shapes and name in shapes else \
                self._buffers[name].max_shape
            if shape != tuple(self.context.get_tensor_shape(name)):
                self.context.set_input_shape(name, shape)
            actual_in[name] = shape
        for name in self.input_names:
            buf = self._buffers[name]
            nbytes = int(np.prod(actual_in[name])) * buf.dtype.itemsize
            _check(_cudart.cudaMemcpyAsync(buf.dev_ptr, buf.host_ptr, nbytes,
                                           _CUDA_MEMCPY_H2D, self._stream), 'H2D copy')
        if not self.context.execute_async_v3(self._stream.value):
            raise RuntimeError('TensorRT execute_async_v3 failed')
        result = {}
        for name in self.output_names:
            buf = self._buffers[name]
            shape = tuple(self.context.get_tensor_shape(name))
            nbytes = int(np.prod(shape)) * buf.dtype.itemsize
            _check(_cudart.cudaMemcpyAsync(buf.host_ptr, buf.dev_ptr, nbytes,
                                           _CUDA_MEMCPY_D2H, self._stream), 'D2H copy')
            result[name] = buf.host.reshape(-1)[:int(np.prod(shape))].reshape(shape)
        _check(_cudart.cudaStreamSynchronize(self._stream), 'cudaStreamSynchronize')
        return result

    def input_view(self, name, shape):
        """Writable view of the leading part of an input buffer, for dynamic
        shapes: fill it, then infer(shapes={name: shape})."""
        buf = self._buffers[name]
        n = int(np.prod(shape))
        return buf.host.reshape(-1)[:n].reshape(shape)

    def close(self):
        for buf in self._buffers.values():
            buf.free()
        self._buffers = {}
        if self._stream.value:
            _cudart.cudaStreamDestroy(self._stream)
            self._stream = ctypes.c_void_p()
        self.context = None
        self.engine = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
