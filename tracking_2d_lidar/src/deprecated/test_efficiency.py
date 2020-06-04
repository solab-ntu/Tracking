import numpy as np
import time

def test_time():
    _time = time.time()

    test_3()

    duration = time.time() - _time
    print(duration)

def test_1():
    a = np.random.rand(300,300)
    b = np.random.rand(300,300)

    c = np.dot(a,b)

def test_2():
    a = np.random.rand(150,150)
    b = np.random.rand(150,150)

    c = np.dot(a,b)

def test_3():
    a = np.random.rand(6,6)
    b = np.random.rand(6,300)

    c = np.dot(a,b)

if __name__ == "__main__":
    test_time()