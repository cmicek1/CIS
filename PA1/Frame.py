import scipy.linalg as scialg

class Frame:
    """
    Class for representing a coordinate frame transformation.
    """
    def __init__(self, r, p):
        """
        Initialize F = [r, p]

        :param r: The rotation matrix of the frame transformation
        :param p: The translation vector of the frame transformation

        :type r: numpy.array([numpy.float64][]), N x N (usually 3 x 3)
        :type p: numpy.array([numpy.float64]), N x 1
        """
        self.r = r
        self.p = p

    @property
    def inv(self):
        """
        The inverse transformation
        :return: A Frame transformation with components [r, p] corresponding to the inverse of the current
                 transformation
        :rtype: Frame
        """
        r_inv = scialg.inv(self.r)
        return Frame(r_inv, -r_inv.dot(self.p))

    def compose(self, f):
        """
        Frame composition with another frame f

        :param f: The Frame to compose with
        ::type f: Frame

        :return: A Frame transformation with components corresponding to the composition of the components of the
                 current frame with those of f
        :rtype: Frame
        """
        return Frame(self.r.dot(f.r), self.r.dot(f.p) + self.p)
