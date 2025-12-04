


class Pathfinder:
    pather = None

    @classmethod
    def get_pather():
        if Pathfinder.pather == None:
            Pathfinder.pather = Pathfinder(5)
    
    def __init__(self, method):
        pass

