#include <Novice.h>
#include <cmath>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <imgui.h>
#include <algorithm>

const char kWindowTitle[] = "LD2B_01_オノ_フミカ_MT3_";

// 3次元ベクトルの構造体
struct Vector3 { float x, y, z; };
// 4ｘ4行列の構造体
struct Matrix4x4 {
	float m[4][4];
};
// 球
struct Sphere {
	Vector3 center; // 中心点
	float radius; // 半径
};
// AABB
struct AABB {
	Vector3 min; //最小点
	Vector3 max; //最大点
};

// ベクトルの内席
float Dot(const Vector3& v1, const Vector3& v2) {
	float result = {};
	result = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);

	return result;
};
// 長さ（ノルム）
float Length(const Vector3& v) {
	float result = {};

	result = sqrtf(Dot(v, v));

	return result;
};
// 減産
Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 result = {};

	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;

	return result;
};

// 座標変換
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	// ｗ = 1 がデカルト座標系であるので（ｘ、ｙ、z,1）のベクトルとしてmatrixとの積をとる
	Vector3 result = {};

	result.x =
		vector.x * matrix.m[0][0] +
		vector.y * matrix.m[1][0] +
		vector.z * matrix.m[2][0] +
		1.0f * matrix.m[3][0];

	result.y =
		vector.x * matrix.m[0][1] +
		vector.y * matrix.m[1][1] +
		vector.z * matrix.m[2][1] +
		1.0f * matrix.m[3][1];

	result.z =
		vector.x * matrix.m[0][2] +
		vector.y * matrix.m[1][2] +
		vector.z * matrix.m[2][2] +
		1.0f * matrix.m[3][2];

	float w =
		vector.x * matrix.m[0][3] +
		vector.y * matrix.m[1][3] +
		vector.z * matrix.m[2][3] +
		1.0f * matrix.m[3][3];

	// ベクトルに対して基本的な操作を行う行列でｗが0になることは無い
	assert(w != 0.0f);
	// ｗ＝1がデカルト座標系であるので、ｗ除算することで同次座標をデカルト座標に戻す
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
};
// X軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 result = {};

	result.m[0][0] = 1;
	result.m[1][1] = std::cos(radian);
	result.m[1][2] = std::sin(radian);
	result.m[2][1] = -std::sin(radian);
	result.m[2][2] = std::cos(radian);
	result.m[3][3] = 1;

	return result;
};
// Y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 result = {};

	result.m[0][0] = std::cos(radian);
	result.m[0][2] = -std::sin(radian);
	result.m[1][1] = 1;
	result.m[2][0] = std::sin(radian);
	result.m[2][2] = std::cos(radian);
	result.m[3][3] = 1;

	return result;
};
// Z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 result = {};

	result.m[0][0] = std::cos(radian);
	result.m[0][1] = std::sin(radian);
	result.m[1][0] = -std::sin(radian);;
	result.m[1][1] = std::cos(radian);
	result.m[2][2] = 1;
	result.m[3][3] = 1;

	return result;
};
// 行列の積
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result = {};

	for (int x = 0; x < 4; x++) {
		for (int y = 0; y < 4; y++) {
			for (int a = 0; a < 4; a++) {
				result.m[x][y] += m1.m[x][a] * m2.m[a][y];
			}
		}
	}

	return result;
};
// アフィン変換
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 result = {};

	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 rotateXYZMatrix = Multiply(rotateXMatrix, Multiply(rotateYMatrix, rotateZMatrix));


	result.m[0][0] = scale.x * rotateXYZMatrix.m[0][0];
	result.m[0][1] = scale.x * rotateXYZMatrix.m[0][1];
	result.m[0][2] = scale.x * rotateXYZMatrix.m[0][2];

	result.m[1][0] = scale.y * rotateXYZMatrix.m[1][0];
	result.m[1][1] = scale.y * rotateXYZMatrix.m[1][1];
	result.m[1][2] = scale.y * rotateXYZMatrix.m[1][2];

	result.m[2][0] = scale.z * rotateXYZMatrix.m[2][0];
	result.m[2][1] = scale.z * rotateXYZMatrix.m[2][1];
	result.m[2][2] = scale.z * rotateXYZMatrix.m[2][2];

	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;
	result.m[3][3] = 1;

	return result;
};
// 逆行列
Matrix4x4 Inverse(const Matrix4x4& m) {
	Matrix4x4 result = {};
	float determinant = 0;

	// 行列式 |A| を求める
	determinant =
		(m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3]) + (m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1]) + (m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]) -
		(m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1]) - (m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3]) - (m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]) -
		(m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3]) - (m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1]) - (m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]) +
		(m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1]) + (m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3]) + (m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]) +
		(m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3]) + (m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1]) + (m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]) -
		(m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1]) - (m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3]) - (m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]) -
		(m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0]) - (m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0]) - (m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]) +
		(m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0]) + (m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0]) + (m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0]);

	// 逆行列を求める
	result.m[0][0] = (1 / determinant) *
		((m.m[1][1] * m.m[2][2] * m.m[3][3]) + (m.m[1][2] * m.m[2][3] * m.m[3][1]) + (m.m[1][3] * m.m[2][1] * m.m[3][2]) -
			(m.m[1][3] * m.m[2][2] * m.m[3][1]) - (m.m[1][2] * m.m[2][1] * m.m[3][3]) - (m.m[1][1] * m.m[2][3] * m.m[3][2]));
	result.m[0][1] = (1 / determinant) *
		(-(m.m[0][1] * m.m[2][2] * m.m[3][3]) - (m.m[0][2] * m.m[2][3] * m.m[3][1]) - (m.m[0][3] * m.m[2][1] * m.m[3][2]) +
			(m.m[0][3] * m.m[2][2] * m.m[3][1]) + (m.m[0][2] * m.m[2][1] * m.m[3][3]) + (m.m[0][1] * m.m[2][3] * m.m[3][2]));
	result.m[0][2] = (1 / determinant) *
		((m.m[0][1] * m.m[1][2] * m.m[3][3]) + (m.m[0][2] * m.m[1][3] * m.m[3][1]) + (m.m[0][3] * m.m[1][1] * m.m[3][2]) -
			(m.m[0][3] * m.m[1][2] * m.m[3][1]) - (m.m[0][2] * m.m[1][1] * m.m[3][3]) - (m.m[0][1] * m.m[1][3] * m.m[3][2]));
	result.m[0][3] = (1 / determinant) *
		(-(m.m[0][1] * m.m[1][2] * m.m[2][3]) - (m.m[0][2] * m.m[1][3] * m.m[2][1]) - (m.m[0][3] * m.m[1][1] * m.m[2][2]) +
			(m.m[0][3] * m.m[1][2] * m.m[2][1]) + (m.m[0][2] * m.m[1][1] * m.m[2][3]) + (m.m[0][1] * m.m[1][3] * m.m[2][2]));

	result.m[1][0] = (1 / determinant) *
		(-(m.m[1][0] * m.m[2][2] * m.m[3][3]) - (m.m[1][2] * m.m[2][3] * m.m[3][0]) - (m.m[1][3] * m.m[2][0] * m.m[3][2]) +
			(m.m[1][3] * m.m[2][2] * m.m[3][0]) + (m.m[1][2] * m.m[2][0] * m.m[3][3]) + (m.m[1][0] * m.m[2][3] * m.m[3][2]));
	result.m[1][1] = (1 / determinant) *
		((m.m[0][0] * m.m[2][2] * m.m[3][3]) + (m.m[0][2] * m.m[2][3] * m.m[3][0]) + (m.m[0][3] * m.m[2][0] * m.m[3][2]) -
			(m.m[0][3] * m.m[2][2] * m.m[3][0]) - (m.m[0][2] * m.m[2][0] * m.m[3][3]) - (m.m[0][0] * m.m[2][3] * m.m[3][2]));
	result.m[1][2] = (1 / determinant) *
		(-(m.m[0][0] * m.m[1][2] * m.m[3][3]) - (m.m[0][2] * m.m[1][3] * m.m[3][0]) - (m.m[0][3] * m.m[1][0] * m.m[3][2]) +
			(m.m[0][3] * m.m[1][2] * m.m[3][0]) + (m.m[0][2] * m.m[1][0] * m.m[3][3]) + (m.m[0][0] * m.m[1][3] * m.m[3][2]));
	result.m[1][3] = (1 / determinant) *
		((m.m[0][0] * m.m[1][2] * m.m[2][3]) + (m.m[0][2] * m.m[1][3] * m.m[2][0]) + (m.m[0][3] * m.m[1][0] * m.m[2][2]) -
			(m.m[0][3] * m.m[1][2] * m.m[2][0]) - (m.m[0][2] * m.m[1][0] * m.m[2][3]) - (m.m[0][0] * m.m[1][3] * m.m[2][2]));

	result.m[2][0] = (1 / determinant) *
		((m.m[1][0] * m.m[2][1] * m.m[3][3]) + (m.m[1][1] * m.m[2][3] * m.m[3][0]) + (m.m[1][3] * m.m[2][0] * m.m[3][1]) -
			(m.m[1][3] * m.m[2][1] * m.m[3][0]) - (m.m[1][1] * m.m[2][0] * m.m[3][3]) - (m.m[1][0] * m.m[2][3] * m.m[3][1]));
	result.m[2][1] = (1 / determinant) *
		(-(m.m[0][0] * m.m[2][1] * m.m[3][3]) - (m.m[0][1] * m.m[2][3] * m.m[3][0]) - (m.m[0][3] * m.m[2][0] * m.m[3][1]) +
			(m.m[0][3] * m.m[2][1] * m.m[3][0]) + (m.m[0][1] * m.m[2][0] * m.m[3][3]) + (m.m[0][0] * m.m[2][3] * m.m[3][1]));
	result.m[2][2] = (1 / determinant) *
		((m.m[0][0] * m.m[1][1] * m.m[3][3]) + (m.m[0][1] * m.m[1][3] * m.m[3][0]) + (m.m[0][3] * m.m[1][0] * m.m[3][1]) -
			(m.m[0][3] * m.m[1][1] * m.m[3][0]) - (m.m[0][1] * m.m[1][0] * m.m[3][3]) - (m.m[0][0] * m.m[1][3] * m.m[3][1]));
	result.m[2][3] = (1 / determinant) *
		(-(m.m[0][0] * m.m[1][1] * m.m[2][3]) - (m.m[0][1] * m.m[1][3] * m.m[2][0]) - (m.m[0][3] * m.m[1][0] * m.m[2][1]) +
			(m.m[0][3] * m.m[1][1] * m.m[2][0]) + (m.m[0][1] * m.m[1][0] * m.m[2][3]) + (m.m[0][0] * m.m[1][3] * m.m[2][1]));

	result.m[3][0] = (1 / determinant) *
		(-(m.m[1][0] * m.m[2][1] * m.m[3][2]) - (m.m[1][1] * m.m[2][2] * m.m[3][0]) - (m.m[1][2] * m.m[2][0] * m.m[3][1]) +
			(m.m[1][2] * m.m[2][1] * m.m[3][0]) + (m.m[1][1] * m.m[2][0] * m.m[3][2]) + (m.m[1][0] * m.m[2][2] * m.m[3][1]));
	result.m[3][1] = (1 / determinant) *
		((m.m[0][0] * m.m[2][1] * m.m[3][2]) + (m.m[0][1] * m.m[2][2] * m.m[3][0]) + (m.m[0][2] * m.m[2][0] * m.m[3][1]) -
			(m.m[0][2] * m.m[2][1] * m.m[3][0]) - (m.m[0][1] * m.m[2][0] * m.m[3][2]) - (m.m[0][0] * m.m[2][2] * m.m[3][1]));
	result.m[3][2] = (1 / determinant) *
		(-(m.m[0][0] * m.m[1][1] * m.m[3][2]) - (m.m[0][1] * m.m[1][2] * m.m[3][0]) - (m.m[0][2] * m.m[1][0] * m.m[3][1]) +
			(m.m[0][2] * m.m[1][1] * m.m[3][0]) + (m.m[0][1] * m.m[1][0] * m.m[3][2]) + (m.m[0][0] * m.m[1][2] * m.m[3][1]));
	result.m[3][3] = (1 / determinant) *
		((m.m[0][0] * m.m[1][1] * m.m[2][2]) + (m.m[0][1] * m.m[1][2] * m.m[2][0]) + (m.m[0][2] * m.m[1][0] * m.m[2][1]) -
			(m.m[0][2] * m.m[1][1] * m.m[2][0]) - (m.m[0][1] * m.m[1][0] * m.m[2][2]) - (m.m[0][0] * m.m[1][2] * m.m[2][1]));

	return result;
};
// 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result = {};
	float cot = 1 / (std::tan(fovY / 2));

	result.m[0][0] = (1 / aspectRatio) * cot;
	result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1;
	result.m[3][2] = -nearClip * (farClip / (farClip - nearClip));

	return result;
};
// ビューポート変換行列
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result = {};

	result.m[0][0] = width / 2;
	result.m[1][1] = -(height / 2);
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + (width / 2);
	result.m[3][1] = top + (height / 2);
	result.m[3][2] = minDepth;
	result.m[3][3] = 1;

	return result;
};

// グリッド
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;// 半分
	const uint32_t kSubdivision = 10;// 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);// 1つ分の長さ

	Vector3 zLineStart;
	Vector3 zLineEnd;
	Vector3 xLineStart;
	Vector3 xLineEnd;

	// 奥から手前の線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		// 視点と頂点を求める
		zLineStart = Vector3(xIndex * kGridEvery - kGridHalfWidth, 0.0f, kGridHalfWidth);
		zLineEnd = Vector3(xIndex * kGridEvery - kGridHalfWidth, 0.0f, -kGridHalfWidth);

		// 変換
		Matrix4x4 worldMatrixStart = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, zLineStart);
		Matrix4x4 worldMatrixEnd = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, zLineEnd);

		Matrix4x4 wvpMatrixStart = Multiply(worldMatrixStart, viewProjectionMatrix);
		Matrix4x4 wvpMatrixEnd = Multiply(worldMatrixEnd, viewProjectionMatrix);

		Vector3 nbcVertexStart = Transform(Vector3{}, wvpMatrixStart);
		Vector3 screenStartPoint = Transform(nbcVertexStart, viewportMatrix);
		Vector3 nbcVertexEnd = Transform(Vector3{}, wvpMatrixEnd);
		Vector3 screenEndPoint = Transform(nbcVertexEnd, viewportMatrix);

		// 描画
		if (xIndex == 5) {
			Novice::DrawLine((int)screenStartPoint.x, (int)screenStartPoint.y, (int)screenEndPoint.x, (int)screenEndPoint.y, BLACK);
		}
		else {
			Novice::DrawLine((int)screenStartPoint.x, (int)screenStartPoint.y, (int)screenEndPoint.x, (int)screenEndPoint.y, 0xAAAAAAFF);
		}
	}

	// 左から右
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		// 視点と頂点を求める
		xLineStart = Vector3(kGridHalfWidth, 0.0f, xIndex * kGridEvery - kGridHalfWidth);
		xLineEnd = Vector3(-kGridHalfWidth, 0.0f, xIndex * kGridEvery - kGridHalfWidth);

		// 変換
		Matrix4x4 worldMatrixStart = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, xLineStart);
		Matrix4x4 worldMatrixEnd = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, xLineEnd);

		Matrix4x4 wvpMatrixStart = Multiply(worldMatrixStart, viewProjectionMatrix);
		Matrix4x4 wvpMatrixEnd = Multiply(worldMatrixEnd, viewProjectionMatrix);

		Vector3 nbcVertexStart = Transform(Vector3{}, wvpMatrixStart);
		Vector3 screenStartPoint = Transform(nbcVertexStart, viewportMatrix);
		Vector3 nbcVertexEnd = Transform(Vector3{}, wvpMatrixEnd);
		Vector3 screenEndPoint = Transform(nbcVertexEnd, viewportMatrix);

		// 描画
		if (xIndex == 5) {
			Novice::DrawLine((int)screenStartPoint.x, (int)screenStartPoint.y, (int)screenEndPoint.x, (int)screenEndPoint.y, BLACK);
		}
		else {
			Novice::DrawLine((int)screenStartPoint.x, (int)screenStartPoint.y, (int)screenEndPoint.x, (int)screenEndPoint.y, 0xAAAAAAFF);
		}
	}

}
// 球
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewPortMatrix, uint32_t color) {
	float pi = float(M_PI);
	const uint32_t kSudivision = 10; //分割数
	const float kLatEvery = pi / kSudivision; //緯度分割一つ分の角度
	const float kLonEvery = (pi * 2) / kSudivision; //緯度分割二つ分の角度
	// world座標系でのa,b,cを求める
	Vector3 a, b, c;

	// 緯度の方向に分割
	for (uint32_t latindex = 0; latindex < kSudivision; ++latindex) {
		sphere;
		float lat = -pi / 2.0f + kLatEvery * latindex; //現在の緯度

		// 経度の方向に分割
		for (uint32_t lonIndex = 0; lonIndex < kSudivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery; //現在の経度
			// ワールド座標系での頂点を求める
			a = {
				sphere.center.x + sphere.radius * (cosf(lat) * cosf(lon)),
				sphere.center.y + sphere.radius * sinf(lat),
				sphere.center.z + sphere.radius * (cosf(lat) * sinf(lon))
			};
			b = {
				sphere.center.x + sphere.radius * (cosf(lat + kLatEvery) * cosf(lon)),
				sphere.center.y + sphere.radius * sinf(lat + kLatEvery),
				sphere.center.z + sphere.radius * (cosf(lat + kLatEvery) * sin(lon))
			};
			c = {
				sphere.center.x + sphere.radius * (cosf(lat) * cosf(lon + kLonEvery)),
				sphere.center.y + sphere.radius * sinf(lat),
				sphere.center.z + sphere.radius * (cos(lat) * sinf(lon + kLonEvery))
			};

			// 変換
			Matrix4x4 worldMatrixA = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, a);
			Matrix4x4 worldMatrixB = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, b);
			Matrix4x4 worldMatrixC = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, c);

			Matrix4x4 wvpMatrixA = Multiply(worldMatrixA, viewProjectionMatrix);
			Matrix4x4 wvpMatrixB = Multiply(worldMatrixB, viewProjectionMatrix);
			Matrix4x4 wvpMatrixC = Multiply(worldMatrixC, viewProjectionMatrix);

			Vector3 nbcVertexA = Transform(Vector3{}, wvpMatrixA);
			Vector3 nbcVertexB = Transform(Vector3{}, wvpMatrixB);
			Vector3 nbcVertexC = Transform(Vector3{}, wvpMatrixC);

			Vector3 screenA = Transform(nbcVertexA, viewPortMatrix);
			Vector3 screenB = Transform(nbcVertexB, viewPortMatrix);
			Vector3 screenC = Transform(nbcVertexC, viewPortMatrix);

			// 描画
			Novice::DrawLine((int)screenA.x, (int)screenA.y, (int)screenB.x, (int)screenB.y, color);
			Novice::DrawLine((int)screenA.x, (int)screenA.y, (int)screenC.x, (int)screenC.y, color);
		}
	}
}
// AABB描画
void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 topLeftFront = { aabb.min.x,aabb.max.y,aabb.min.z };
	Vector3 topRightFront = { aabb.max.x,aabb.max.y,aabb.min.z };
	Vector3 topLeftBehind = { aabb.min.x,aabb.max.y,aabb.max.z };
	Vector3 topRightBehind = { aabb.max.x,aabb.max.y,aabb.max.z };

	Vector3 downLeftFront = { aabb.min.x,aabb.min.y,aabb.min.z };
	Vector3 downRightFront = { aabb.max.x,aabb.min.y,aabb.min.z };
	Vector3 downLeftBehind = { aabb.min.x,aabb.min.y,aabb.max.z };
	Vector3 downRightBehind = { aabb.max.x,aabb.min.y,aabb.max.z };

	// nbc変換
	Vector3 nbcTopLF = Transform(topLeftFront, viewProjectionMatrix);
	Vector3 nbcTopRF = Transform(topRightFront, viewProjectionMatrix);
	Vector3 nbcTopLB = Transform(topLeftBehind, viewProjectionMatrix);
	Vector3 nbcTopRB = Transform(topRightBehind, viewProjectionMatrix);

	Vector3 nbcDownLF = Transform(downLeftFront, viewProjectionMatrix);
	Vector3 nbcDownRF = Transform(downRightFront, viewProjectionMatrix);
	Vector3 nbcDownLB = Transform(downLeftBehind, viewProjectionMatrix);
	Vector3 nbcDownRB = Transform(downRightBehind, viewProjectionMatrix);

	// スクリーン変換
	Vector3 screenTopLF = Transform(nbcTopLF, viewportMatrix);
	Vector3 screenTopRF = Transform(nbcTopRF, viewportMatrix);
	Vector3 screenTopLB = Transform(nbcTopLB, viewportMatrix);
	Vector3 screenTopRB = Transform(nbcTopRB, viewportMatrix);

	Vector3 screenDownLF = Transform(nbcDownLF, viewportMatrix);
	Vector3 screenDownRF = Transform(nbcDownRF, viewportMatrix);
	Vector3 screenDownLB = Transform(nbcDownLB, viewportMatrix);
	Vector3 screenDownRB = Transform(nbcDownRB, viewportMatrix);

	// 描画
	Novice::DrawLine((int)screenTopLF.x, (int)screenTopLF.y, (int)screenTopRF.x, (int)screenTopRF.y, color); // 上の面
	Novice::DrawLine((int)screenTopRF.x, (int)screenTopRF.y, (int)screenTopRB.x, (int)screenTopRB.y, color);
	Novice::DrawLine((int)screenTopRB.x, (int)screenTopRB.y, (int)screenTopLB.x, (int)screenTopLB.y, color);
	Novice::DrawLine((int)screenTopLB.x, (int)screenTopLB.y, (int)screenTopLF.x, (int)screenTopLF.y, color);

	Novice::DrawLine((int)screenDownLF.x, (int)screenDownLF.y, (int)screenDownRF.x, (int)screenDownRF.y, color); // 下の面
	Novice::DrawLine((int)screenDownRF.x, (int)screenDownRF.y, (int)screenDownRB.x, (int)screenDownRB.y, color);
	Novice::DrawLine((int)screenDownRB.x, (int)screenDownRB.y, (int)screenDownLB.x, (int)screenDownLB.y, color);
	Novice::DrawLine((int)screenDownLB.x, (int)screenDownLB.y, (int)screenDownLF.x, (int)screenDownLF.y, color);

	Novice::DrawLine((int)screenTopLF.x, (int)screenTopLF.y, (int)screenDownLF.x, (int)screenDownLF.y, color); // 縦線
	Novice::DrawLine((int)screenTopRF.x, (int)screenTopRF.y, (int)screenDownRF.x, (int)screenDownRF.y, color);
	Novice::DrawLine((int)screenTopLB.x, (int)screenTopLB.y, (int)screenDownLB.x, (int)screenDownLB.y, color);
	Novice::DrawLine((int)screenTopRB.x, (int)screenTopRB.y, (int)screenDownRB.x, (int)screenDownRB.y, color);
}

// 衝突判定
bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	// 最近接点を求める
	Vector3 closestPoint{
		std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
		std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),
		std::clamp(sphere.center.z,aabb.min.z,aabb.max.z), };

	// 最近接点と球の中心との距離を求める
	float distance = Length(Subtract(closestPoint, sphere.center));
	// 距離が半径より小さければ衝突
	if (distance <= sphere.radius) {
		return true;
	}

	return false;
}


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };
	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };
	int color = WHITE;
	int kWindowWidth = 1280;
	int kWindowHeight = 720;

	AABB aabb{
		.min{-0.5f,-0.5f,-0.5f},
		.max{0.0f,0.0f,0.0f}
	};
	Sphere sphere = { {0.0f,0.0f,0.0f},1.0f };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		Matrix4x4 cameraWorldMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraWorldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		if (IsCollision(aabb, sphere)) {
			color = RED;
		}
		else {
			color = WHITE;
		}

		// minとmaxが入れ替わらないようにする
		aabb.min.x = (std::min)(aabb.min.x, aabb.max.x); //x
		aabb.max.x = (std::max)(aabb.min.x, aabb.max.x);
		aabb.min.y = (std::min)(aabb.min.y, aabb.max.y); //y
		aabb.max.y = (std::max)(aabb.min.y, aabb.max.y);
		aabb.min.z = (std::min)(aabb.min.z, aabb.max.z); //z
		aabb.max.z = (std::max)(aabb.min.z, aabb.max.z);


		ImGui::DragFloat3("aabb01.min", &aabb.min.x, 0.01f);
		ImGui::DragFloat3("aabb01.max", &aabb.max.x, 0.01f);
		ImGui::DragFloat3("sphere.center", &sphere.center.x, 0.01f);
		ImGui::DragFloat("sphere.radius", &sphere.radius, 0.01f);
		ImGui::DragFloat3("CamereRotate", &cameraRotate.x, 0.01f);
		ImGui::DragFloat3("cameraTranslate", &cameraTranslate.x, 0.01f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		DrawGrid(worldViewProjectionMatrix, viewportMatrix);
		DrawSphere(sphere, worldViewProjectionMatrix, viewportMatrix, WHITE);
		DrawAABB(aabb, worldViewProjectionMatrix, viewportMatrix, color);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
