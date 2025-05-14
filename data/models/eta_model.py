import joblib
from sklearn.ensemble import RandomForestRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, KFold, cross_val_score
from sklearn.metrics import mean_absolute_error

class ETAModel:
    def __init__(self):
        self.pipeline = Pipeline([
            ('scaler', StandardScaler()),
            ('model', RandomForestRegressor(n_estimators=100, random_state=42))
        ])
        self.trained = False

    def train(self, X, y):
        X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)
        self.pipeline.fit(X_train, y_train)
        y_pred = self.pipeline.predict(X_val)
        mae = mean_absolute_error(y_val, y_pred)
        print(f"Validation MAE: {mae:.2f} seconds")
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

    def save(self, path="data/models/eta_predictor.pkl"):
        joblib.dump(self.pipeline, path)

    def load(self, path="data/models/eta_predictor.pkl"):
        self.pipeline = joblib.load(path)
        self.trained = True


