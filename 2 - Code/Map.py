class Point:
        def __init__(self,y,x,val):
            self.x=x
            self.y=y
            self.val=val

        def __str__(self):
            return "Point[x:{} y:{} val:{}]".format(self.x,self.y,self.val)

class Map:


    BLOCK = 0
    SPACE = 1
    length = 0
    width = 0
    array = []
    checkpoints=[]

    def __init__(self,array,length,width,checkpoints):
        self.array = array
        self.length = length
        self.width = width
        self.checkpoints=checkpoints

    @classmethod
    def fromFile(cls,file):
        map = open(file,"r")
        x=0
        y=0
        row=[]
        length=0;
        array=[]
        checkpoints=[]
        while True:
                c = map.read(1)
                #if EOF
                if c=='':
                        return cls(array,y,length,checkpoints)
                elif c=='*':
                        row.append(Point(y,x,cls.BLOCK))
                        # print("{},{} > '{}'".format(y,x,c))
                elif c==' ':
                        row.append(Point(y,x,cls.SPACE))
                        # print("{},{} > '{}'".format(y,x,c))
                else :
                        if c != '\n':
                            checkpoint =  Point(y,x,c)
                            row.append(checkpoint)
                            checkpoints.append(checkpoint)
                            # print("{},{} > '{}'".format(y,x,c))
                x+=1
                if c=='\n':
                    length=x-1
                    x=0
                    y+=1
                    array.append(row)
                    row=[]

    def __str__(self):
        line=""
        for i in range(self.length):
                for j in range(self.width):
                            # print("{} {}".format(i,j))
                            if self.array[i][j].val == 0 :
                                if (i+j)  % 2:
                                    c=chr(9619)+chr(9619)+chr(9619)
                                else:
                                    c=chr(9608)+chr(9608)+chr(9608)
                            elif self.array[i][j].val == 1 :
                                if (i+j) % 2:
                                    c=chr(9617)+chr(9617)+chr(9617)
                                else:
                                    # c=chr(9618)+chr(9618)+chr(9618)
                                    c = "   "
                            elif isinstance(self.array[i][j].val,str):
                                    c = " " + self.array[i][j].val + " "
                            else:
                                 c = " "+chr(self.array[i][j].val)+" "
                            line+= c
                            # str(self.array[i][j].val)+" "
                line+="\n"
        return line

    def getAdjacent_UP(self,point):
            if self.array[point.y-1][point.x]:
                return self.array[point.y-1][point.x]
            else :
                return Point(point.y-1,point.x,self.BLOCK)
    def getAdjacent_DOWN(self,point):
            if self.array[point.y+1][point.x]:
                return self.array[point.y+1][point.x]
            else :
                return Point(point.y+1,point.x,self.BLOCK)
    def getAdjacent_RIGHT(self,point):
            if self.array[point.y][point.x+1]:
                return self.array[point.y][point.x+1]
            else :
                return Point(point.y,point.x+1,self.BLOCK)
    def getAdjacent_LEFT(self,point):
            if self.array[point.y][point.x-1]:
                return self.array[point.y][point.x-1]
            else :
                return Point(point.y,point.x-1,self.BLOCK)

    def getManhattenDistance(self,point1,point2):
            return abs(point1.x-point2.x)+abs(point2.y-point1.y)

    def getCheckpointPairs(self):

        #order alphapatically
        for i in range(len(self.checkpoints)):
            for j in range(i+1,len(self.checkpoints)):
                if self.checkpoints[i].val > self.checkpoints[j].val:
                    tmp = self.checkpoints[i]
                    self.checkpoints[i] = self.checkpoints[j]
                    self.checkpoints[j] = tmp
       #find pairs
        l=[]
        for i in range(len(self.checkpoints)):
            for j in range(i+1,len(self.checkpoints)):
                l.append([self.checkpoints[i],self.checkpoints[j]])
        return l
