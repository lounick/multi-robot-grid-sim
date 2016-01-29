class Target:
    id = 0
    classification = "None"
    pos = [0, 0]

    def __init__(self, uid, pos):
        self.id = uid
        self.pos = pos