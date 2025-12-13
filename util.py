import math


def length(vector):
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2)

def unit_vector(vector):
    l = length(vector)
    if l == 0:
        return (0, 0)
    return (vector[0] / l, vector[1] / l)

def dir_average(a, b):
    if length(a) == 0:
        return b
    if length(b) == 0:
        return a
    a_u = unit_vector(a)
    b_u = unit_vector(b)

    c = (a_u[0] + b_u[0], a_u[1] + b_u[1])
    if length(c) == 0:
        return (0,0,0)
    return unit_vector(c)
    