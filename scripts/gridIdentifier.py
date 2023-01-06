class utilfunctions:
    def __init__(self):
        self.h = 1600
        self.w = 1600
        self.rs = 16
        self.cs = 16
        self.initializeParams()

        
    def initializeParams(self):
        self.delta = 1600/16
        self.initialI = self.delta/2
        self.initialJ = self.delta/2

    def updateDelta(self, delta):
        self.delta = delta

    def ij2rc(self,i,j):
        # print(i,j)
        r = int((i-self.initialI+self.delta/2)//self.delta)
        c = int((j-self.initialJ+self.delta/2)//self.delta)
        return(r,c)

    def rc2ij(self, r,c):
        i = int(self.initialI + self.delta*r)
        j = int(self.initialJ + self.delta*c)
        return (i,j)

    def ij2xy(self,i,j):
        factor = 2.4384/1600
        x = (i)*factor
        y = (self.h-j)*factor
        return (x,y)

    
