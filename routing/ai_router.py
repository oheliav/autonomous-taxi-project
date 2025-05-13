 #ai_router.py
 
import joblib
import os

class ETAEstimator:
    def __init__(self, model_path="data/models/eta_predictor.pkl"):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found at: {model_path}")
        self.model = joblib.load(model_path)
        
    def predict_eta(self, feature_vector):
        return float(self.model.predict([feature_vector])[0])
    
    def predict_batch(self, feature_matrix):
        return self.model.predict(feature_matrix).tolist()
        