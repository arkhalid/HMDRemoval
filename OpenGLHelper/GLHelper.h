#ifndef _GLHELPER_H_
#define _GLHELPER_H_

#include <GL/glew.h>
#include <string>
#include "algebra3.h"
#include <vector>
#include <opencv2\opencv.hpp>
void die();
void dieOnGLError(const string& errMessage);
void dieOnInvalidIndex(int k, const std::string& err);

inline void* BUFFER_OFFSET(int i) {
	return ((void*)i);
}

void setUniformInt(GLuint program, const std::string& uniformName, int v);
void setUniformFloat(GLuint program, const std::string& uniformName, float v);
void setUniformVec2(GLuint program, const std::string& uniformName, const algebra3::vec2& v);
void setUniformVec3(GLuint program, const std::string& uniformName, const algebra3::vec3& v);
void setUniformVec4(GLuint program, const std::string& uniformName, const algebra3::vec4& v);
void setUniformMat3(GLuint program, const std::string& uniformName, const algebra3::mat3& v);
void setUniformMat4(GLuint program, const std::string& uniformName, const algebra3::mat4& v);

GLuint createShaderProgram(const std::string& vShaderFilename, const std::string& fShaderFilename);
bool ReadPLY(string filename, vector<float> &vertices, vector<float> &colors, vector<unsigned int> &indices);

class RenderObject
{
public:
	RenderObject();
	RenderObject(vector<float> *vertices, vector<float> *colors, vector<unsigned int> *indices);
	RenderObject(vector<float> *vertices, vector<float> *colors);
	bool AddTexCoords(vector<float> *TexCoords);
	~RenderObject();

	int GetIndCount();

	int GetVertCount();
	vector<unsigned int>* GetIndices();
	vector<float>* GetVertices();
	vector<float>* GetColors();
	vector<float>* GetNormals();
	GLuint GetVAO();
	GLuint GetVBO();
	GLuint GetCBO();
	GLuint GetElemBuff();
	bool GetHasFaces();
private:
	bool m_hasVerts;
	bool m_hasColors;
	bool m_hasNormals;
	bool m_hasFaces;
	bool m_hasTexCoords;
	int m_indCount;
	int m_vertCount;
	vector<unsigned int>* m_indices;
	vector<float>* m_vertices;
	vector<float>* m_colors;
	vector<float>* m_normals;
	vector<float>* m_texCoords;
	GLuint m_vao;
	GLuint m_vbo;
	GLuint m_cbo;
	GLuint m_tbo;
	GLuint m_elembuf;

};

class Texture
{
public:
	Texture();
	~Texture();

	Texture(cv::Mat image, GLint minMagFiler, GLint wrapMode);
	bool UpdateTexture(cv::Mat image);
	GLuint GetTexture();
private:
	GLuint m_tex;
	int m_originalWidth;
	int m_originalHeight;
};

#endif
