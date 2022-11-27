class PID():
    def __init__(self,P=2,I=0,D=0.2):
        self.P = P
        self.I = I
        self.D = D
        self.error = 0
        self.error_p1 = 0
        self.error_p2 = 0
        self.sum_error = 0

    def PD_controller(self,target, current):
        self.error_p1 = self.error_p2
        self.error_p1 = self.error
        Ditem = self.error_p1-self.error_p2
        self.error = target-current
        U = self.P * self.error + self.D * Ditem
        return  current+U












