class ControlSystem:
    def __init__(self, kp=0.00185, setpoint=-22.452118490490875):
        self.kp = kp
        self.setpoint = setpoint

    def control_p(self, ubicacion_linea):
        error = self.setpoint + ubicacion_linea
        return error * self.kp

    @staticmethod
    def saturate(raw_steering, major, less):
        if raw_steering <= less:
            return less
        elif raw_steering >= major:
            return major
        return raw_steering

    def set_kp(self, new_kp):
        """
        Method to modify the kp value.
        
        :param new_kp: The new value for kp
        """
        self.kp = new_kp

    def get_kp(self):
        """
        Method to retrieve the current kp value.
        
        :return: The current kp value
        """
        return self.kp

# Usage example:
# control_system = ControlSystem()
# steering = control_system.control_p(some_line_location)
# saturated_steering = control_system.saturate(steering, max_value, min_value)
# 
# # Modify kp
# control_system.set_kp(0.00225)
# 
# # Get current kp value
# current_kp = control_system.get_kp()
# print(f"Current kp value: {current_kp}")