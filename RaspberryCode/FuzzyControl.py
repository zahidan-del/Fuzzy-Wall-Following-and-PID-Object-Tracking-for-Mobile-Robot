import numpy as np

class WallFollower:

    def _init_(self, target_distance=100, log_path="wall_log.txt"):
        self.last_distance = target_distance
        self.target_distance = target_distance
        self.log_path = log_path
    
    def fuzzy_control(self, distance, return_error=False):
        """Fuzzy logic controller untuk navigasi dinding"""
        error = distance - self.target_distance
        delta_error = error - (self.last_distance - self.target_distance)
        self.last_distance = distance

        # Fungsi keanggotaan error
        e_nb = max(0, min(1, (-error - 40) / 40.0)) if error < -40 else 0
        e_ns = max(0, min(1, (-error) / 40.0, (error + 80) / 40.0)) if -80 < error < 0 else 0
        e_ze = max(0, min(1, (error + 40) / 40.0, (-error + 40) / 40.0)) if -40 < error < 40 else 0
        e_ps = max(0, min(1, (error) / 40.0, (-error + 80) / 40.0)) if 0 < error < 80 else 0
        e_pb = max(0, min(1, (error - 40) / 40.0)) if error > 40 else 0

        # Fungsi keanggotaan delta error
        de_neg = max(0, -delta_error / 20.0) if delta_error < 0 else 0
        de_zero = max(0, 1 - abs(delta_error) / 20.0)
        de_pos = max(0, delta_error / 20.0) if delta_error > 0 else 0

        # Rule base
        rule_weights = [
            min(e_nb, de_neg), min(e_nb, de_zero), min(e_nb, de_pos),
            min(e_ns, de_neg), min(e_ns, de_zero), min(e_ns, de_pos),
            min(e_ze, de_neg), min(e_ze, de_zero), min(e_ze, de_pos),
            min(e_ps, de_neg), min(e_ps, de_zero), min(e_ps, de_pos),
            min(e_pb, de_neg), min(e_pb, de_zero), min(e_pb, de_pos)
        ]
        
        rule_outputs = [
            40, 30, 20, 25, 15, 5, 10, 0, -10, -5, -15, -25, -20, -30, -40
        ]

        # Hitung output
        numerator = sum(w * o for w, o in zip(rule_weights, rule_outputs))
        denominator = sum(rule_weights)
        
        output = numerator / denominator if denominator != 0 else 0

        # Logging
        with open(self.log_path, "a") as f:
            f.write(f"Fuzzy Error: {error:.2f}\n")
            f.write(f"Fuzzy OUT: {output:.2f}\n")

        if return_error:
            return output, error
        return output