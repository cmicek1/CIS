
class BoundingSphere:

    def __init__(self, c, r):
        """
        Initialize sphere with given center and radius.

        :param c: The center of the bounding sphere
        :param r: The radius of the bounding sphere

        :type c: numpy.array([numpy.float64]), 3x1
        :type r: numpy.float64
        """
        self.c = c
        self.r = r
