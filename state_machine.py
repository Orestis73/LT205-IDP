class navigation:

    def __init__(self):

        #self.stack = ["pd", "pu","ou", "od"] #purple down, purple up, orange up orange down
        #self.destinations = {"red", "yellow", "green", "blue","RETURN"}
        self.reel_count = 0
        self.previous_stack = None
        self.current_stack = "pd"
        self.current_destination = None
        #self.switch_stack = None
        #self.is_picking = False
        self.stack_reel_count_test = {"pd":[0,3,"red"], "pu":[0,None,"yellow"],"ou":[0,None,"green"], "od":[0,None,"red"]}
        self.stack_reel_count = self.stack_reel_count_test  #[searched stack number, picked reels number]
        #self.orientation = 0

        self.scanning = False
        self.nav_state = None

        self.routes = {
            "pd":[self.grab_from_pd],#self.go_pd, self.grab_from_pd, self.pd_pu], 
            "pu":[ self.grab_from_pu],#self.go_pu, self.grab_from_pu, self.pu_ou],
            "ou":[self.grab_from_ou],#self.go_ou, self.grab_from_ou, self.ou_od],
            "od":[self.grab_from_od]#self.go_od, self.grab_from_od]
        }

    def stack_decider(self):
        for i in range(len(self.stack_reel_count)):
            stack = list(self.stack_reel_count.keys())[i]
            if self.stack_reel_count[stack][0] >= 6:
                continue
            else:
                self.previous_stack = self.current_stack
                self.current_stack = stack
                break
            
    def destination_decider(self):
        '''resistance detecting logic'''
        self.reel_count += 1
        #self.is_picking = True
        #if self.reel_count >= 4:
            #self.current_destination = "RETURN"
        #else:  #resistance logic here
            #just for test!
        self.current_destination = self.stack_reel_count[self.current_stack][2]#self.current_destination = sth
            
        return None
    
    def route_executor(self):
        self.stack_decider()
        #self.destination_decider()
        if self.nav_state == "picking":
            func = self.routes[self.current_stack][0]
            print(f"picking from {self.current_stack} to {self.current_destination}")
            mission = func()
            self.nav_state = "go main"
        if self.nav_state == "go main":
            if self.reel_count >= 4:
                mission = self.go_back()
                self.nav_state = "END"
            else:
                mission = self.go_to_main_route()
                self.nav_state = None
        if not self.nav_state:
            self.stack_reel_count = self.stack_reel_count_test
            print("Go main route") 
            mission = self.main_route()
        if self.nav_state == "END":
            mission = ["END"]
        
        return mission
    '''
    def run(self):
        while True:
            #self.stack_decider()
            self.route_executor()
            print(self.is_picking, self.current_stack, self.current_destination)
            if #reel_detected:
                self.go_to_pick()
                self.stack_decider()
                self.destination_decider()'''


    #----------------------------mission list generation functions-------------------------------
    def go_to_pick(self):
        None

    def main_route(self):
        m = []
        #scan pd
        m.append("start_scan")
        for i in range(6):
            m.append("straight")
        m.append("end_scan")
        #going to pu
        m.append("straight")
        for i in range(4):
            m.append("left")
        #scan pu
        m.append("start_scan")
        for i in range(6):
            m.append("straight")
        m.append("end_scan")
        #going to ou
        m.append("180")
        for i in range(6): 
            m.append("straight")
        m.append("right")
        m.append("straight")
        m.append("right")
        #scan ou
        m.append("start_scan")
        for i in range(6):
            m.append("straight")
        m.append("end_scan")
        #going to od
        m.append("180")
        for i in range(6): 
            m.append("straight")
        for i in range(4):
            m.append("left")
        #scan od
        m.append("start_scan")
        for i in range(6):
            m.append("straight")
        m.append("end_scan")
        return m

    def grab_from_pd(self):
        m = []
        for i in range(self.stack_reel_count[self.current_stack][0]-1):
            m.append("straight")
        destinations = {
            "red":["straight"],
            "yellow":["right","left"],
            "green":["right","straight", "straight", "left"],
            "blue": ["right","straight", "straight", "straight", "left"]
        }
        a = destinations[self.current_destination]
        m = m + a
        m.append("180")
        return m  

    def grab_from_pu(self):
        m = []
        for i in range(self.stack_reel_count[self.current_stack][0]-1):
            m.append("straight")
        for i in range(4):
            m.append("right")
        for i in range(7):
            m.append("straight")
        destinations = {
            "red":["straight"],
            "yellow":["right","left"],
            "green":["right","straight", "straight", "left"],
            "blue": ["right","straight", "straight", "straight", "left"]
        }
        a = destinations[self.current_destination]
        m = m + a
        m.append("180")
        return m
    
    def grab_from_ou(self):
        m = []
        for i in range(self.stack_reel_count[self.current_stack][0]-1):
            m.append("straight")
        for i in range(4):
            m.append("left")
        for i in range(7):
            m.append("straight")
        destinations = {
            "blue":["straight"],
            "green":["left","right"],
            "yellow":["left","straight", "straight", "right"],
            "red": ["left","straight", "straight", "straight", "right"]
        }
        a = destinations[self.current_destination]
        m = m + a
        m.append("180")
        return m

    def grab_from_od(self):
        m = []
        m.append('180')
        for i in range(6 - self.stack_reel_count[self.current_stack][0]+1):
            m.append("straight")
        destinations = {
            "blue":["straight"],
            "green":["left","right"],
            "yellow":["left","straight", "straight", "right"],
            "red": ["left","straight", "straight", "straight", "right"]
        }
        a = destinations[self.current_destination]
        m = m + a
        m.append("180")
        return m  
        
    def go_to_main_route(self):
        destinations = {
            "red":["straight"],
            "yellow":["right","left"],
            "green":["right","straight", "straight", "left"],
            "blue": ["right","straight", "straight", "straight", "left"]
        }
        m = destinations[self.current_destination]
        return m

    def go_back(self):
        destinations = {
            "red":["left", "straight", "left"],
            "yellow":["left","left"],
            "green":["right","right"],
            "blue": ["right","straight", "right"]
        }
        m = destinations[self.current_destination]
        return m

        


