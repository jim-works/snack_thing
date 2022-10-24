import rospy

class SnackBowl(object):
    def __init__(self):
        self.max_volume = 1
        self.current_volume = 0
        self.wasted = 0

    def add_snack(self, volume):
        rospy.loginfo(f"Added {volume} snack volume!")
        self.current_volume += volume
        if self.current_volume > self.max_volume: #overflow!
            self.wasted += self.current_volume - self.max_volume
            self.current_volume = self.max_volume
            rospy.loginfo(f"We have now wasted {self.wasted} snack volume :(")
    def full(self):
        return self.max_volume <= self.current_volume

#
    #snack_bowl = SnackBowl()
    #snack_dispenser = SnackDispenser(snack_bowl, 0.5, 1)
#
#
class SnackDispenser:
    #creates a snack dispenser that drops into snack_bowl, that drops rate units/sec of snacks into it
    #there is a period of delay seconds where the snack is in the air: it will only increase the number of snacks in the bowl after delay seconds. 
    def __init__(self, snack_bowl: SnackBowl, delay: float, rate: float):
        self.snack_bowl = snack_bowl
        self.delay = delay
        self.rate = rate
        self.check_interval = 0.05
        self.dispense_time_elapsed = 0
        self.start_dispensing()
        self.dispensing = True
        rospy.Timer(rospy.Duration(self.check_interval), self.try_dispense) #check often if we are under a snack dispenser
        
    def robot_under(self):
        return self.dispensing

    def start_dispensing(self):
        self.dispense_time_elapsed = 0
        self.dispensing = True

    def stop_dispensing(self):
        self.dispensing = False

    def should_dispense(self):
        return self.snack_bowl.current_volume <= self.snack_bowl.max_volume

    def try_dispense(self, event):
        if self.dispensing:
            self.dispense_time_elapsed += event.last_duration
            if not self.should_dispense():
                self.dispensing = False
            self.snack_bowl.drop_snack(self.rate*event.last_duration, self.delay)
        if self.snack_bowl.full():
            self.stop_dispensing()
    def drop_snack(self, volume):
        rospy.Timer(rospy.Duration(self.delay), lambda event: self.snack_bowl.add_snack(volume), oneshot=True)