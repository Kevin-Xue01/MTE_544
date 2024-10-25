from utilities import ControllerGains, Config, ControllerType, Logger

class PID_ctrl:
    def __init__(self, config: dict[ControllerGains, float], filename, history_length=3):
        self.history_length=history_length
        self.history=[]
        self.type = Config.CONTROLLER_TYPE

        # Controller gains
        self.kp = config[ControllerGains.KP] # proportional gain
        self.kv = config[ControllerGains.KD] # derivative gain
        self.ki = config[ControllerGains.KI] # integral gain
        
        self.logger=Logger(filename + "_errors.csv")
    
    def update(self, stamped_error: tuple[float, int], status) -> float:
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error: tuple[float, int]): #
        latest_error = stamped_error[0]
        latest_error_timestamp = stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error
        
        dt_avg = 0.0
        error_dot = 0.0
        
        for i in range(1, len(self.history)):
            
            t0 = self.history[i-1][1]
            t1 = self.history[i][1]
            
            dt = (t1 - t0) / 1e9
            dt_avg += dt

            error_dot += (self.history[i][0] - self.history[i-1][0]) / dt
            
        error_dot /= len(self.history) - 1
        dt_avg /= len(self.history) - 1
        
        sum_ = sum([val[0] for val in self.history])
        error_int = sum_ * dt_avg
        
        self.logger.log_values((latest_error, error_dot, error_int, latest_error_timestamp))
        
        if self.type == ControllerType.P:
            return self.kp * latest_error
        
        elif self.type == ControllerType.PD:
            return self.kp * latest_error + self.kv * error_dot
        
        elif self.type == ControllerType.PI:
            return self.kp * latest_error + self.ki * error_int
        
        elif self.type == ControllerType.PID:
            return self.kp * latest_error + self.kv * error_dot + self.ki * error_int
