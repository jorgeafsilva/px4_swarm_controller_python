class SecondOrderSmoother:
    def __init__(self, alpha: float):
        """
        Initialize the second-order smoothing filter.
        
        Parameters:
        - alpha (float): Smoothing factor (0 < alpha ≤ 1).
        """
        if not (0 < alpha <= 1):
            raise ValueError("Alpha must be in the range (0, 1].")
        
        self.alpha = alpha
        self.s1 = None  # First-order smoothed value
        self.s2 = None  # Second-order smoothed value
    
    def update(self, x: float) -> float:
        """
        Update the filter with a new input signal and return the smoothed value.
        
        Parameters:
        - x (float): New input signal value.
        
        Returns:
        - float: Smoothed output.
        """
        if self.s1 is None:  # First-time initialization
            self.s1 = x
            self.s2 = x
        else:
            self.s1 = self.alpha * x + (1 - self.alpha) * self.s1
            self.s2 = self.alpha * self.s1 + (1 - self.alpha) * self.s2
        
        return self.s2  # Return second-order smoothed value
