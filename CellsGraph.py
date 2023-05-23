from Cell import Cell
from controller import Supervisor

class CellsGraph:
    def __init__(self, supervisor):
        self.worldCellsDict = {}

        self.createCells(supervisor)

    def createCells(self,supervisor):

        maxCounter = 27

        for counter in range(1,maxCounter+1):
            newCellName = "C"+ str(counter)

            cellNode = supervisor.getFromDef(newCellName)
            cellPosition = cellNode.getField('translation').value

            newCell = Cell(newCellName,cellPosition)

            self.worldCellsDict[newCellName] = newCell

        print("Number cells: " + str(len(self.worldCellsDict)))

        for c in self.worldCellsDict.keys():

            tempX = str(self.worldCellsDict[c].posX)
            tempY = str(self.worldCellsDict[c].posY)
            tempZ = str(self.worldCellsDict[c].posZ)
            print("NAME: " + c + "\n" + " X -> " + tempX + " Y -> " + tempY + " Z -> " + tempZ + "\n\n")

