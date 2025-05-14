import joblib
from xgboost import XGBRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import KFold, cross_val_score
from sklearn.metrics import mean_absolute_error

class XGBETAModel:
    def __init__(self):
        self.pipeline = Pipeline([
            ('scaler', StandardScaler()),
            ('model', XGBRegressor(
                n_estimators=200,
                max_depth=6,
                learning_rate=0.1,
                objective='reg:squarederror',
                random_state=42,
                n_jobs=-1
            ))
        ])
        self.trained = False

    def train(self, X, y):
        self.pipeline.fit(X, y)
        self.trained = True

    def cross_validate(self, X, y, folds=20):
        kf = KFold(n_splits=folds, shuffle=True, random_state=42)
        scores = cross_val_score(self.pipeline, X, y, cv=kf, scoring='neg_mean_absolute_error')
        mean_mae = -scores.mean()
        print(f"{folds}-fold Cross-Validation MAE: {mean_mae:.2f} seconds")

    def predict(self, X_input):
        if not self.trained:
            raise RuntimeError("Model must be trained or loaded before predicting.")
        return self.pipeline.predict([X_input])[0]

    def save(self, path="data/models/xgb_eta_predictor.pkl"):
        joblib.dump(self.pipeline, path)

    def load(self, path="data/models/xgb_eta_predictor.pkl"):
        self.pipeline = joblib.load(path)
        self.trained = True
