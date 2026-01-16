import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
import joblib

# 카메라 기반 객체 검출 결과를 사람 몸체 기준 50% 이상이 나올때 사람으로 학습
# 학습된 모델은 이미지 디버깅 노드에서 사용하기 위해 joblib 파일로 저장
data_path = "/home/cho/lch_ws/src/turtle_pkg/config/person_dataset.csv"
df = pd.read_csv(data_path).dropna()

feature_cols = [
    "yolo_conf",
    "center_x_norm",
    "area_ratio",
    "aspect_ratio",
    "w_ratio",
    "h_ratio",
]
X = df[feature_cols].to_numpy(dtype=np.float32)
y = df["label"].to_numpy(dtype=np.int64)

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.25, random_state=42, stratify=y
)

model = Pipeline([
    ("scaler", StandardScaler()),
    ("clf", LogisticRegression(max_iter=2000, class_weight="balanced"))
])
    
model.fit(X_train, y_train)

pred = model.predict(X_test)
proba = model.predict_proba(X_test)[:, 1]

print("Confusion Matrix:\n", confusion_matrix(y_test, pred))
print("\nReport:\n", classification_report(y_test, pred, digits=4))

out_path = "/home/cho/lch_ws/src/turtle_pkg/config/person_gate_model.joblib"
joblib.dump({"model": model, "feature_cols": feature_cols}, out_path)
print("Saved:", out_path)

