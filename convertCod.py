import numpy as np

#convert from coordinate used by nao documentation to that used in robotic books
def convCod2book(arr):
    arr = arr + np.array([0, 50, 85])
    newArr = np.array([-arr[2], arr[1], arr[0]])
    return newArr

#convert from coordinate used in robotic books to that used by nao documentation
def convCod2doc(arr):
    newArr = np.array([arr[2], arr[1], -arr[0]])
    newArr = newArr +np.array([0, -50, -85])
    return newArr
