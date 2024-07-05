import numpy

X: numpy.ndarray = numpy.random.random((10, 3))
Y: numpy.ndarray = numpy.random.random((10, 3))

Z: numpy.ndarray = (X - Y)
print(X)
print(Y)
print(Z)
print(numpy.linalg.norm(Z, axis=1))
