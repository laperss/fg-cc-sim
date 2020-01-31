import numpy as np


# For now, this is a PD controller
class ControllerPID(object):
    def __init__(self, vref, p_uav, d_uav, p_ugv, d_ugv):
        self.vref = vref
        self.p_uav = p_uav
        self.p_ugv = p_ugv
        self.d_uav = d_uav
        self.d_ugv = d_ugv

    def get_control(self, deltax, deltay, deltau, deltav=0.0):
        # HEADING
        # heading_uav = (0 - 0.02*deltay - 0.0001*deltav)*180/3.14      # [deg]
        # heading_ugv = (0 + 0.005*deltay - 0.00001*deltav)*180/3.14    # [deg]
        # uav_ctrl.setpoint['heading'] = max(min(5, heading_uav), -5)
        # ugv_ctrl.setpoint['heading'] = max(min(5, heading_ugv), -5)

        # VELOCITY
        v_uav = self.vref - self.p_uav*deltax - self.d_uav*deltau  # m/s
        v_ugv = self.vref + self.p_ugv*deltax + self.d_ugv*deltau  # m/s

        return v_uav, v_ugv
