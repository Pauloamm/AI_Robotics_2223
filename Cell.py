class Cell:
    def __init__(self,name,position):
        self.name = name
        self.posX = position[0]
        self.posY = position[1]
        self.posZ = position[2]

    def DefineAdjacentCells(self,adjacentCells):
        self.adjacentCells = adjacentCells

    def GetAdjacentCells(self):
        return self.adjacentCells

