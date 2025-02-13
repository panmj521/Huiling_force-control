from .arm_state import ArmState

class ControllerManager:
    def __init__(self, state: ArmState):
        self.state = state
        self.controllers = {}
        self.current_controller = None

    def add_controller(self, controller_class, name, config_path):
        if name not in self.controllers:
            self.controllers[name] = controller_class(name,self.state,config_path)
        else:
            raise ValueError(f"Controller {name} already exists")

    def remove_controller(self, name):
        if name in self.controllers:
            del self.controllers[name]
            if self.current_controller == self.controllers.get(name):
                self.current_controller = None
        else:
            raise ValueError(f"Controller {name} does not exist")

    def switch_controller(self, name):
        if name in self.controllers:
            self.current_controller = self.controllers[name]
        else:
            raise ValueError(f"Controller {name} does not exist")
        
    def __getattr__(self, name):
        if self.current_controller:
            method = getattr(self.current_controller, name, None)
            if method:
                return method
            else:
                raise AttributeError(f"'{type(self.current_controller).__name__}' object has no attribute '{name}'")
        else:
            raise RuntimeError("No current controller is set")


    def step(self,dt):
        if self.current_controller:
            self.current_controller.step(dt)