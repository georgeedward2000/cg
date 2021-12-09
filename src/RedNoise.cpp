#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <ModelTriangle.h>
#include <RayTriangleIntersection.h>
#include <TextureMap.h>
#include <Utils.h>
#include <fstream>
#include <glm/glm.hpp>
#include <math.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#define WIDTH 640
#define HEIGHT 480
#define MAX_DEPTH 10
enum Render { WIREFRAME, RASTERISED, RAYTRACED };
float clamp(float x, float mini, float maxi) {
  return fmax(fmin(x, maxi), mini);
}
void orbit(glm::vec3 &cameraPosition, glm::mat3 &cameraOrientation,
           float &angleTotal, float &frontWalk, float unit) {

  if (angleTotal <= -2 * M_PI) {
    if (frontWalk < M_PI_4 / 5) {
      glm::vec3 step = (cameraPosition - glm::vec3(0, 0, 0)) * 0.05f; // origin
      frontWalk += glm::length(step);
      cameraPosition = cameraPosition - (unit * step);
      return;
    } else {
      frontWalk = 0;
      angleTotal = 0;
    }
  } else {
    float angle = -0.05;
    angleTotal += angle;
    glm::mat3 rotationMatrix =
        glm::mat3(glm::vec3(cos(angle), 0, -sin(angle)), glm::vec3(0, 1, 0),
                  glm::vec3(sin(angle), 0, cos(angle)));
    cameraPosition = rotationMatrix * cameraPosition;
  }
}
bool IsTextured(ModelTriangle triangle) {
  return triangle.texturePoints[0].x != 0 || triangle.texturePoints[0].y != 0 ||
         triangle.texturePoints[1].x != 0 || triangle.texturePoints[1].y != 0 ||
         triangle.texturePoints[2].x != 0 || triangle.texturePoints[2].y != 0;
}
glm::mat3 lookAt(glm::vec3 cameraPosition, glm::vec3 point) {
  glm::vec3 cameraForward = glm::normalize(point - cameraPosition);
  glm::vec3 vertical = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
  glm::vec3 cameraRight = glm::normalize(glm::cross(cameraForward, vertical));
  glm::vec3 cameraUp = glm::normalize(glm::cross(cameraRight, cameraForward));
  glm::mat3 view(cameraRight, cameraUp, -cameraForward);
  return view;
}
void printVec3(glm::vec3 vec) {
  std::cout << vec.x << ' ' << vec.y << ' ' << vec.z << std::endl;
}

bool isIntersectionValid(glm::vec3 solution) {
  return (solution.y >= 0.0f && solution.y <= 1.0f) &&
         (solution.z >= 0.0f && solution.z <= 1.0f) && (solution.x >= 0.0f) &&
         (solution.y + solution.z <= 1.0f);
}
int getClosestIntersection(glm::vec3 cameraPosition, glm::vec3 rayDirection,
                           std::vector<ModelTriangle> &modelTriangle,
                           RayTriangleIntersection &closestIntersection,
                           int &trianglesIntersected) {
  int index = 0;
  closestIntersection.distanceFromCamera = std::numeric_limits<float>::max();
  for (ModelTriangle &triangle : modelTriangle) {
    index += 1;
    glm::vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
    glm::vec3 e1 = triangle.vertices[2] - triangle.vertices[0];
    glm::vec3 SPVector = cameraPosition - triangle.vertices[0];
    glm::mat3 DEMatrix(-rayDirection, e0, e1);
    glm::vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;
    if (!isIntersectionValid(possibleSolution))
      continue;

    trianglesIntersected += 1;
    float distance = possibleSolution.x;
    if (distance < closestIntersection.distanceFromCamera) {
      glm::vec3 intersectionPoint =
          cameraPosition + possibleSolution.x * rayDirection;
      closestIntersection.distanceFromCamera = distance;
      closestIntersection.intersectedTriangle = triangle;
      closestIntersection.intersectionPoint = intersectionPoint;
      closestIntersection.triangleIndex = index - 1;
    }
  }
  if (closestIntersection.distanceFromCamera ==
      std::numeric_limits<float>::max()) {
    return false;
  }
  return true;
}
glm::vec3 colorToVec(Colour color) {
  return glm::vec3(color.red, color.green, color.blue);
}
float diffuseLight(std::vector<glm::vec3> lightSources,
                   std::vector<ModelTriangle> &modelTriangle,
                   RayTriangleIntersection closestIntersection,
                   glm::vec3 cameraPosition, glm::vec3 pointNormal) {
  float diffuseLight = 0;
  float lightStrength = 2.0f;
  for (auto &it : lightSources) {
    glm::vec3 lightRay = it - closestIntersection.intersectionPoint;
    float magnitude = glm::length(lightRay);
    lightRay = glm::normalize(lightRay);
    RayTriangleIntersection lightClosestIntersection;
    int trianglesIntersected = 0;
    if (getClosestIntersection(
            closestIntersection.intersectionPoint + pointNormal * 0.00001f,
            lightRay, modelTriangle, lightClosestIntersection,
            trianglesIntersected)) {
      if (lightClosestIntersection.distanceFromCamera <= magnitude) {
        continue;
      }
    }
    float angleOfIncidence = fmax(glm::dot(lightRay, pointNormal), 0.0f);
    float surfaceArea = (4 * M_PI * std::pow(magnitude, 2));
    float currentDiffuseLight = 0;
    currentDiffuseLight = (lightStrength * angleOfIncidence) / surfaceArea;
    currentDiffuseLight = fmax(currentDiffuseLight, 0.0f);
    diffuseLight += currentDiffuseLight;
  }

  diffuseLight = fmax(diffuseLight, 0.0f);
  diffuseLight /= lightSources.size();
  return diffuseLight;
}
float specularLight(glm::vec3 cameraPosition,
                    std::vector<glm::vec3> lightSources,
                    RayTriangleIntersection closestIntersection,
                    glm::vec3 pointNormal) {
  float specularLight = 0;
  for (auto &it : lightSources) {
    glm::vec3 lightRay = it - closestIntersection.intersectionPoint;
    glm::vec3 Ri = -lightRay;
    glm::vec3 Rr =
        glm::normalize(Ri - 2.0f * pointNormal * glm::dot(pointNormal, Ri));
    glm::vec3 V =
        glm::normalize(cameraPosition - closestIntersection.intersectionPoint);
    float dotVvRr = clamp(glm::dot(V, Rr), 0.0f, 1.0f);
    float currentSpecularLight = std::pow(dotVvRr, 256);
    specularLight += 2.0f * currentSpecularLight;
  }
  specularLight /= lightSources.size();
  return specularLight;
}

void computeTriangleNormal(ModelTriangle &triangle) {
  glm::vec3 e1 = triangle.vertices[1] - triangle.vertices[0];
  glm::vec3 e2 = triangle.vertices[2] - triangle.vertices[0];
  triangle.normal = -1.0f * glm::normalize(glm::cross(e2, e1));
}
bool parseOBJ(const char *path, std::vector<ModelTriangle> &modelTriangle,
              std::vector<glm::vec3> &vertices,
              std::vector<glm::vec3> &verticesNormals,
              std::vector<glm::vec2> &verticesTexture,
              std::vector<std::vector<int>> &trianglesVerticesIndexes,
              std::unordered_map<std::string, Colour> colourMap,
              TextureMap textureMap, float scale) {
  FILE *file = fopen(path, "r");
  if (file == NULL) {
    printf("Impossible to open the file !\n");
    return false;
  }
  bool isTextured = false;
  while (true) {
    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF)
      break; // EOF = End Of File. Quit the loop.
    char colour[128];
    if (strcmp(lineHeader, "v") == 0) {
      glm::vec3 vertex;
      float x, y, z;
      fscanf(file, "%f %f %f\n", &x, &y, &z);
      vertex.x = x * scale;
      vertex.y = y * scale;
      vertex.z = z * scale;
      verticesNormals.push_back(glm::vec3(0.f));
      vertices.push_back(vertex);
      isTextured = false;
    } else if (strcmp(lineHeader, "vt") == 0) {
      glm::vec2 vertex;
      float x, y;
      fscanf(file, "%f %f\n", &x, &y);
      vertex.x = int(textureMap.width * x) % textureMap.width;
      vertex.y = int(textureMap.height * y) % textureMap.height;
      verticesTexture.push_back(vertex);
      isTextured = true;
    } else if (strcmp(lineHeader, "f") == 0) {
      int x, y, z;
      ModelTriangle triangle;
      if (isTextured) {
        int t1, t2, t3;
        fscanf(file, "%d/%d %d/%d %d/%d\n", &x, &t1, &y, &t2, &z, &t3);
        --t1;
        --t2;
        --t3;
        triangle.texturePoints[0] =
            TexturePoint(verticesTexture[t1].x, verticesTexture[t1].y);
        triangle.texturePoints[1] =
            TexturePoint(verticesTexture[t2].x, verticesTexture[t2].y);
        triangle.texturePoints[2] =
            TexturePoint(verticesTexture[t3].x, verticesTexture[t3].y);
      } else {
        fscanf(file, "%d/ %d/ %d/\n", &x, &y, &z);
      }
      --x;
      --y;
      --z;
      triangle.vertices[0] = vertices[x];
      triangle.vertices[1] = vertices[y];
      triangle.vertices[2] = vertices[z];
      triangle.colour = colourMap[colour];

      computeTriangleNormal(triangle);
      verticesNormals[x] += triangle.normal;
      verticesNormals[y] += triangle.normal;
      verticesNormals[z] += triangle.normal;
      trianglesVerticesIndexes.push_back(std::vector<int>{x, y, z});
      modelTriangle.push_back(triangle);
    } else if (strcmp(lineHeader, "usemtl") == 0) {
      fscanf(file, "%s\n", colour);
    }
  }
  return true;
}
bool parseMTL(const char *path,
              std::unordered_map<std::string, Colour> &colourMap,
              TextureMap &textureMap) {
  std::vector<glm::vec3> vertices;
  FILE *file = fopen(path, "r");
  if (file == NULL) {
    printf("Impossible to open the file !\n");
    return false;
  }
  while (true) {
    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF)
      break; // EOF = End Of File. Quit the loop.
    char newmtl[128];
    if (strcmp(lineHeader, "newmtl") == 0) {
      fscanf(file, "%s\n", newmtl);
    } else if (strcmp(lineHeader, "Kd") == 0) {
      float x, y, z;
      fscanf(file, "%f %f %f\n", &x, &y, &z);
      colourMap[newmtl] =
          Colour(newmtl, int(255.0f * x), int(255.0f * y), int(255.0f * z));
    } else if (strcmp(lineHeader, "map_Kd") == 0) {
      char textureFile[128];
      fscanf(file, "%s\n", textureFile);
      char source[] = "src/";
      strcat(source, textureFile);
      textureMap = TextureMap(source);
    }
  }
  return true;
}
glm::vec3 getSpatialIntersectionPoint(glm::vec3 cameraPosition,
                                      glm::mat3 cameraOrientation,
                                      glm::vec3 canvasPoint,
                                      float focalLength) {
  glm::vec3 vertexPosition;
  vertexPosition.z = -focalLength;
  vertexPosition.y = -(canvasPoint.y - HEIGHT / 2) / 100.0;
  vertexPosition.x = (canvasPoint.x - WIDTH / 2) / 100.0;
  vertexPosition = vertexPosition * glm::inverse(cameraOrientation);
  vertexPosition -= cameraPosition;
  return vertexPosition;
}
CanvasPoint getCanvasIntersectionPoint(glm::vec3 cameraPosition,
                                       glm::mat3 cameraOrientation,
                                       glm::vec3 vertexPosition,
                                       TexturePoint texturePoint,
                                       float focalLength) {
  CanvasPoint canvasPoint;
  glm::vec3 cameraToVertex =
      (vertexPosition - cameraPosition) * cameraOrientation;
  canvasPoint.x =
      300.0 * (-1) * focalLength * cameraToVertex.x / cameraToVertex.z +
      WIDTH / 2;
  canvasPoint.y =
      300.0 * focalLength * cameraToVertex.y / cameraToVertex.z + HEIGHT / 2;
  canvasPoint.depth = cameraToVertex.z;
  canvasPoint.texturePoint = texturePoint;
  return canvasPoint;
}
void handleEvent(SDL_Event event, DrawingWindow &window,
                 glm::vec3 &cameraPosition, glm::mat3 &cameraOrientation,
                 std::vector<std::vector<float>> &depthBuffer, Render &render,
                 std::vector<glm::vec3> &lightSource) {
  if (event.type == SDL_KEYDOWN) {
    if (event.key.keysym.sym == SDLK_LEFT) {
      std::cout << "LEFT" << std::endl;
      cameraPosition -= glm::vec3(0.05, 0, 0);
    } else if (event.key.keysym.sym == SDLK_RIGHT) {
      std::cout << "RIGHT" << std::endl;
      cameraPosition += glm::vec3(0.05, 0, 0);
    } else if (event.key.keysym.sym == SDLK_UP) {
      std::cout << "UP" << std::endl;
      cameraPosition += glm::vec3(0, 0.05, 0);
    } else if (event.key.keysym.sym == SDLK_DOWN) {
      std::cout << "DOWN" << std::endl;
      cameraPosition -= glm::vec3(0, 0.05, 0);
    } else if (event.key.keysym.sym == SDLK_n) {
      std::cout << "n" << std::endl;
      cameraPosition -= glm::vec3(0, 0, 0.1);
    } else if (event.key.keysym.sym == SDLK_m) {
      std::cout << "m" << std::endl;
      cameraPosition += glm::vec3(0, 0, 0.1);
    } // LIGHT SOURCE
    else if (event.key.keysym.sym == SDLK_t) {
      std::cout << "t" << std::endl;
      lightSource[0] += glm::vec3(0.5, 0, 0.0);
    } else if (event.key.keysym.sym == SDLK_g) {
      std::cout << "g" << std::endl;
      lightSource[0] -= glm::vec3(0.5, 0, 0.0);
    } else if (event.key.keysym.sym == SDLK_f) {
      std::cout << "f" << std::endl;
      lightSource[0] += glm::vec3(0, 0.5, 0.0);
    } else if (event.key.keysym.sym == SDLK_h) {
      std::cout << "h" << std::endl;
      lightSource[0] -= glm::vec3(0, 0.5, 0.0);
    } else if (event.key.keysym.sym == SDLK_v) {
      std::cout << "v" << std::endl;
      lightSource[0] += glm::vec3(0, 0.0, 0.5);
    } else if (event.key.keysym.sym == SDLK_b) {
      std::cout << "b" << std::endl;
      lightSource[0] -= glm::vec3(0, 0.0, 0.5);
    }
    // ROTATE ABOUT X AXIS CAMERA POSITION
    else if (event.key.keysym.sym == SDLK_k) {
      std::cout << "K" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, cos(angle), sin(angle)),
                    glm::vec3(0, -sin(angle), cos(angle)));
      cameraPosition = rotationMatrix * cameraPosition;
    } else if (event.key.keysym.sym == SDLK_i) {
      std::cout << "I" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, cos(angle), sin(angle)),
                    glm::vec3(0, -sin(angle), cos(angle)));
      cameraPosition = rotationMatrix * cameraPosition;
    }
    // ROTATE ABOUT Y AXIS CAMERA POSITION
    else if (event.key.keysym.sym == SDLK_l) {
      std::cout << "l" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(cos(angle), 0, -sin(angle)), glm::vec3(0, 1, 0),
                    glm::vec3(sin(angle), 0, cos(angle)));
      cameraPosition = rotationMatrix * cameraPosition;
    } else if (event.key.keysym.sym == SDLK_j) {
      std::cout << "j" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(cos(angle), 0, -sin(angle)), glm::vec3(0, 1, 0),
                    glm::vec3(sin(angle), 0, cos(angle)));
      cameraPosition = rotationMatrix * cameraPosition;
    }
    // ROTATE ABOUT X AXIS CAMERA ORIENTATION
    else if (event.key.keysym.sym == SDLK_s) {
      std::cout << "s" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0.0f, cos(angle), sin(angle)),
                    glm::vec3(0, -sin(angle), cos(angle)));
      cameraOrientation = rotationMatrix * cameraOrientation;
    } else if (event.key.keysym.sym == SDLK_w) {
      std::cout << "w" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, cos(angle), sin(angle)),
                    glm::vec3(0, -sin(angle), cos(angle)));
      cameraOrientation = rotationMatrix * cameraOrientation;
    }
    // ROTATE ABOUT Y AXIS CAMERA ORIENTATION
    else if (event.key.keysym.sym == SDLK_d) {
      std::cout << "d" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(cos(angle), 0, -sin(angle)), glm::vec3(0, 1, 0),
                    glm::vec3(sin(angle), 0, cos(angle)));
      cameraOrientation = rotationMatrix;
    } else if (event.key.keysym.sym == SDLK_a) {
      std::cout << "a" << std::endl;
      float angle = 0.01;
      glm::mat3 rotationMatrix =
          glm::mat3(glm::vec3(cos(angle), 0, -sin(angle)), glm::vec3(0, 1, 0),
                    glm::vec3(sin(angle), 0, cos(angle)));
      cameraOrientation = rotationMatrix;
    } else if (event.key.keysym.sym == SDLK_v) {
      glm::mat3 view = lookAt(cameraPosition, glm::vec3(0, 0, 0));
      cameraOrientation = view;
    } else if (event.key.keysym.sym == SDLK_1) {
      render = WIREFRAME;
    } else if (event.key.keysym.sym == SDLK_2) {
      render = RASTERISED;
    } else if (event.key.keysym.sym == SDLK_3) {
      render = RAYTRACED;
    }
  } else if (event.type == SDL_MOUSEBUTTONDOWN) {
    window.savePPM("output.ppm");
    window.saveBMP("output.bmp");
  }
}
void draw_line(DrawingWindow &window, CanvasPoint from, CanvasPoint to,
               Colour colour, std::vector<std::vector<float>> &depthBuffer) {
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  float zDiff = to.depth - from.depth;
  float numberOfSteps = fmax(std::fabs(xDiff), std::fabs(yDiff));
  float xStepSize = xDiff / numberOfSteps;
  float yStepSize = yDiff / numberOfSteps;
  float zStepSize = zDiff / numberOfSteps;
  uint32_t colourWindow =
      (255 << 24) + (colour.red << 16) + (colour.green << 8) + colour.blue;
  for (float i = 0.0; i < numberOfSteps; ++i) {
    float x = from.x + (xStepSize * i);
    float y = from.y + (yStepSize * i);
    float z = -(from.depth + (zStepSize * i));
    int roundedX = int(x);
    int roundedY = int(y);
    if (z == 0.0) {
      window.setPixelColour(roundedX, roundedY, colourWindow);
    } else if (depthBuffer[roundedX][roundedY] < 1.0 / z) {
      window.setPixelColour(roundedX, roundedY, colourWindow);
      depthBuffer[roundedX][roundedY] = 1.0 / z;
    }
  }
}
void fillUpTriangle(DrawingWindow &window, CanvasTriangle triangle,
                    Colour colour,
                    std::vector<std::vector<float>> &depthBuffer) {
  float invsloap1 =
      (triangle.v1().x - triangle.v0().x) / (triangle.v1().y - triangle.v0().y);
  float invsloap2 =
      (triangle.v2().x - triangle.v0().x) / (triangle.v2().y - triangle.v0().y);
  float Zinvsloap1 = (triangle.v1().depth - triangle.v0().depth) /
                     (triangle.v1().y - triangle.v0().y);
  float Zinvsloap2 = (triangle.v2().depth - triangle.v0().depth) /
                     (triangle.v2().y - triangle.v0().y);
  float curx1 = triangle.v0().x;
  float curx2 = triangle.v0().x;
  float curz1 = triangle.v0().depth;
  float curz2 = triangle.v0().depth;
  for (float line = triangle.v0().y; line < triangle.v1().y; ++line) {
    CanvasPoint from(curx1, line, curz1);
    CanvasPoint to(curx2, line, curz2);
    draw_line(window, from, to, colour, depthBuffer);
    curx1 += invsloap1;
    curx2 += invsloap2;
    curz1 += Zinvsloap1;
    curz2 += Zinvsloap2;
  }
}
void fillDownTriangle(DrawingWindow &window, CanvasTriangle triangle,
                      Colour colour,
                      std::vector<std::vector<float>> &depthBuffer) {
  float invsloap1 =
      (triangle.v2().x - triangle.v0().x) / (triangle.v2().y - triangle.v0().y);
  float invsloap2 =
      (triangle.v2().x - triangle.v1().x) / (triangle.v2().y - triangle.v1().y);
  float Zinvsloap1 = (triangle.v2().depth - triangle.v0().depth) /
                     (triangle.v2().y - triangle.v0().y);
  float Zinvsloap2 = (triangle.v2().depth - triangle.v1().depth) /
                     (triangle.v2().y - triangle.v1().y);
  float curx1 = triangle.v2().x;
  float curx2 = triangle.v2().x;
  float curz1 = triangle.v2().depth;
  float curz2 = triangle.v2().depth;
  for (int line = triangle.v2().y; line > triangle.v0().y; --line) {
    CanvasPoint from(curx1, line, curz1);
    CanvasPoint to(curx2, line, curz2);
    draw_line(window, from, to, colour, depthBuffer);
    curx1 -= invsloap1;
    curx2 -= invsloap2;
    curz1 -= Zinvsloap1;
    curz2 -= Zinvsloap2;
  }
}
std::vector<CanvasPoint> interpolatePoints(CanvasPoint start, CanvasPoint end,
                                           int steps) {
  std::vector<CanvasPoint> result;
  float stepX = (end.x - start.x) / (steps - 1);
  float stepY = (end.y - start.y) / (steps - 1);
  float stepDepth = (end.depth - start.depth) / (steps - 1);
  CanvasPoint temp = start;
  result.push_back(temp);
  for (int i = 0; i < steps - 1; i++) {
    temp.x = temp.x + stepX;
    temp.y = temp.y + stepY;
    temp.depth = temp.depth + stepDepth;
    result.push_back(temp);
  }
  return result;
}
void BarycentricRasterizer(CanvasPoint p, CanvasPoint a, CanvasPoint b,
                           CanvasPoint c, float &v, float &w, float &u) {
  glm::vec2 aVec(a.x, a.y);
  glm::vec2 bVec(b.x, b.y);
  glm::vec2 cVec(c.x, c.y);
  glm::vec2 pVec(p.x, p.y);
  glm::vec2 v0 = bVec - aVec;
  glm::vec2 v1 = cVec - aVec;
  glm::vec2 v2 = pVec - aVec;
  float d00 = glm::dot(v0, v0);
  float d01 = glm::dot(v0, v1);
  float d11 = glm::dot(v1, v1);
  float d20 = glm::dot(v2, v0);
  float d21 = glm::dot(v2, v1);
  float denom = d00 * d11 - d01 * d01;
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;
}
std::vector<TexturePoint>
interpolateTexturePoints(TexturePoint start, TexturePoint end, int steps) {
  std::vector<TexturePoint> result;
  float stepX = (end.x - start.x) / (steps - 1);
  float stepY = (end.y - start.y) / (steps - 1);
  TexturePoint temp = start;
  result.push_back(temp);
  for (int i = 0; i < steps - 1; i++) {
    temp.x = temp.x + stepX;
    temp.y = temp.y + stepY;
    result.push_back(temp);
  }
  return result;
}
float getQ(glm::vec3 &v0, glm::vec3 &v1, glm::vec3 &v2) {
  return (v2.x - v0.x) * (v1.y - v0.y) - (v2.y - v0.y) * (v1.x - v0.x);
}
void draw_textured_triangle(DrawingWindow &window, CanvasTriangle triangle,
                            TextureMap textureMap,
                            std::vector<std::vector<float>> &depthBuffer) {
  // Sort the values
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (triangle.vertices[i].y > triangle.vertices[j].y) {
        std::swap(triangle.vertices[i], triangle.vertices[j]);
      }
    }
  }
  // Get middle point
  float ratio =
      (triangle.v1().y - triangle.v0().y) / (triangle.v2().y - triangle.v0().y);
  CanvasPoint v3(triangle.v0().x + (triangle.v2().x - triangle.v0().x) * ratio,
                 triangle.v1().y,
                 triangle.v0().depth +
                     (triangle.v2().depth - triangle.v0().depth) * ratio);
  v3.texturePoint = TexturePoint(
      triangle.v0().texturePoint.x +
          (triangle.v2().texturePoint.x - triangle.v0().texturePoint.x) * ratio,
      triangle.v0().texturePoint.y +
          (triangle.v2().texturePoint.y - triangle.v0().texturePoint.y) *
              ratio);
  std::vector<CanvasPoint> left = interpolatePoints(
      triangle.v0(), triangle.v1(), triangle.v1().y - triangle.v0().y + 2);
  std::vector<CanvasPoint> right = interpolatePoints(
      triangle.v0(), v3, triangle.v1().y - triangle.v0().y + 2);
  for (int i = 0; i < left.size(); i++) {
    int steps = std::fabs(left[i].x - right[i].x);
    std::vector<CanvasPoint> points =
        interpolatePoints(left[i], right[i], steps + 2);
    for (int tp = 0; tp < points.size(); tp++) {
      float v = 0;
      float w = 0;
      float u = 0;
      BarycentricRasterizer(points[tp], triangle.v0(), triangle.v1(),
                            triangle.v2(), v, w, u);
      if (u < 0 || v < 0 || w < 0) {
        continue;
      }
      glm::vec2 textureVec0 =
          glm::vec2(triangle.v0().texturePoint.x, triangle.v0().texturePoint.y);
      glm::vec2 textureVec1 =
          glm::vec2(triangle.v1().texturePoint.x, triangle.v1().texturePoint.y);
      glm::vec2 textureVec2 =
          glm::vec2(triangle.v2().texturePoint.x, triangle.v2().texturePoint.y);
      float bary0 = u / -triangle.v0().depth;
      float bary1 = v / -triangle.v1().depth;
      float bary2 = w / -triangle.v2().depth;
      float denom = bary0 + bary1 + bary2;
      glm::vec3 newRatio = glm::vec3(bary0, bary1, bary2) / denom;
      int x_coord =
          int(newRatio.x * textureVec0.x + newRatio.y * textureVec1.x +
              newRatio.z * textureVec2.x) %
          textureMap.width;
      int y_coord =
          int(newRatio.x * textureVec0.y + newRatio.y * textureVec1.y +
              newRatio.z * textureVec2.y) %
          textureMap.height;
      uint32_t col =
          textureMap.pixels[int((y_coord * textureMap.width) + x_coord)];
      float z = -points[tp].depth;
      if (z == 0) {
        window.setPixelColour(round(points[tp].x), round(points[i].y), col);
      } else if (depthBuffer[round(points[tp].x)][round(points[i].y)] <
                 1.0 / z) {
        depthBuffer[round(points[tp].x)][round(points[i].y)] = 1.0 / z;

        window.setPixelColour(round(points[tp].x), round(points[i].y), col);
      }
    }
  }
  std::vector<CanvasPoint> left2 = interpolatePoints(
      triangle.v2(), triangle.v1(), triangle.v2().y - triangle.v1().y + 2);
  std::vector<CanvasPoint> right2 = interpolatePoints(
      triangle.v2(), v3, triangle.v2().y - triangle.v1().y + 2);
  for (int i = 0; i < left2.size(); i++) {
    int steps = std::fabs(left2[i].x - right2[i].x);

    std::vector<CanvasPoint> points =
        interpolatePoints(left2[i], right2[i], steps + 2);
    for (int tp = 0; tp < points.size(); tp++) {
      float v = 0;
      float w = 0;
      float u = 0;
      BarycentricRasterizer(points[tp], triangle.v0(), triangle.v1(),
                            triangle.v2(), v, w, u);
      if (u < 0 || v < 0 || w < 0) {
        continue;
      }
      glm::vec2 textureVec0 =
          glm::vec2(triangle.v0().texturePoint.x, triangle.v0().texturePoint.y);
      glm::vec2 textureVec1 =
          glm::vec2(triangle.v1().texturePoint.x, triangle.v1().texturePoint.y);
      glm::vec2 textureVec2 =
          glm::vec2(triangle.v2().texturePoint.x, triangle.v2().texturePoint.y);
      float bary0 = u / -triangle.v0().depth;
      float bary1 = v / -triangle.v1().depth;
      float bary2 = w / -triangle.v2().depth;
      float denom = bary0 + bary1 + bary2;
      glm::vec3 newRatio = glm::vec3(bary0, bary1, bary2) / denom;
      int x_coord =
          int(newRatio.x * textureVec0.x + newRatio.y * textureVec1.x +
              newRatio.z * textureVec2.x) %
          textureMap.width;
      int y_coord =
          int(newRatio.x * textureVec0.y + newRatio.y * textureVec1.y +
              newRatio.z * textureVec2.y) %
          textureMap.height;
      uint32_t col =
          textureMap.pixels[int((y_coord * textureMap.width) + x_coord)];
      float z = -points[tp].depth;
      if (z == 0) {
        window.setPixelColour(round(points[tp].x), round(points[i].y), col);
      } else if (depthBuffer[round(points[tp].x)][round(points[i].y)] <
                 1.0 / z) {
        depthBuffer[round(points[tp].x)][round(points[i].y)] = 1.0 / z;
        window.setPixelColour(round(points[tp].x), round(points[i].y), col);
      }
    }
  }
}
void draw_filled_triangle(DrawingWindow &window, CanvasTriangle triangle,
                          Colour colour,
                          std::vector<std::vector<float>> &depthBuffer) {
  // Sort the values
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (triangle.vertices[i].y > triangle.vertices[j].y) {
        std::swap(triangle.vertices[i], triangle.vertices[j]);
      }
    }
  }
  // Get middle point
  float ratio =
      (triangle.v1().y - triangle.v0().y) / (triangle.v2().y - triangle.v0().y);
  CanvasPoint v3(triangle.v0().x + (triangle.v2().x - triangle.v0().x) * ratio,
                 triangle.v1().y,
                 triangle.v0().depth +
                     (triangle.v2().depth - triangle.v0().depth) * ratio);
  // Up triangle
  CanvasTriangle up_triangle(triangle.v0(), triangle.v1(), v3);
  fillUpTriangle(window, up_triangle, colour, depthBuffer);
  // Down triangle
  CanvasTriangle down_triangle(triangle.v1(), v3, triangle.v2());
  fillDownTriangle(window, down_triangle, colour, depthBuffer);
  // Fill the middle line
  draw_line(window, triangle.v1(), v3, colour, depthBuffer);
  // Draw stroke
  draw_line(window, triangle.v0(), triangle.v1(), colour, depthBuffer);
  draw_line(window, triangle.v1(), triangle.v2(), colour, depthBuffer);
  draw_line(window, triangle.v2(), triangle.v0(), colour, depthBuffer);
}
CanvasPoint toCanvasPoint(glm::vec3 point) {
  return CanvasPoint(point.x, point.y, point.z);
}
void drawRasterisedScene(DrawingWindow &window, glm::vec3 cameraPosition,
                         glm::mat3 cameraOrientation, float focalPoint,
                         std::vector<ModelTriangle> modelTriangle,
                         std::vector<std::vector<float>> &depthBuffer,
                         TextureMap TextureMap) {
  for (auto &it : depthBuffer) {
    std::fill(it.begin(), it.end(), 0);
  }
  window.clearPixels();
  for (ModelTriangle &it : modelTriangle) {
    CanvasPoint first_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[0], it.texturePoints[0],
        focalPoint);
    CanvasPoint second_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[1], it.texturePoints[1],
        focalPoint);
    CanvasPoint third_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[2], it.texturePoints[2],
        focalPoint);
    CanvasTriangle triangle(first_vertex, second_vertex, third_vertex);
    if (IsTextured(it)) {
      draw_textured_triangle(window, triangle, TextureMap, depthBuffer);
    } else {
      Colour triangleColour(it.colour.red, it.colour.green, it.colour.blue);
      draw_filled_triangle(window, triangle, triangleColour, depthBuffer);
    }
  }
}
uint32_t getColourFromTriangle(ModelTriangle triangle) {
  return (255 << 24) + (int(triangle.colour.red) * 255 << 16) +
         (int(triangle.colour.green) * 255 << 8) +
         int(triangle.colour.blue) * 255;
}
uint32_t getPixel(float brightness, Colour colour) {
  colour.red *= brightness;
  colour.blue *= brightness;
  colour.green *= brightness;
  uint32_t red = uint32_t(clamp(colour.red, 0.f, 255.f));
  uint32_t green = uint32_t(clamp(colour.green, 0.f, 255.f));
  uint32_t blue = uint32_t(clamp(colour.blue, 0.f, 255.f));
  return (255 << 24) + (red << 16) + (green << 8) + blue;
}
uint32_t getPixel(float brightness, uint32_t colour) {
  Colour c((colour >> 16) & 0xff, (colour >> 8) & 0xff, colour & 0xff);
  return getPixel(brightness, c);
}
void barycentricInterpolation(
    RayTriangleIntersection intersection,
    std::vector<glm::vec3> &verticesNormals,
    std::vector<std::vector<int>> &trianglesVerticesIndexes,
    glm::vec3 &pointNormal, glm::vec2 &pointTexture) {
  int triangleIndex = intersection.triangleIndex;
  glm::vec3 v0Normal =
      verticesNormals[trianglesVerticesIndexes[triangleIndex][0]];
  glm::vec3 v1Normal =
      verticesNormals[trianglesVerticesIndexes[triangleIndex][1]];
  glm::vec3 v2Normal =
      verticesNormals[trianglesVerticesIndexes[triangleIndex][2]];
  glm::vec2 v0Texture(intersection.intersectedTriangle.texturePoints[0].x,
                      intersection.intersectedTriangle.texturePoints[0].y);
  glm::vec2 v1Texture(intersection.intersectedTriangle.texturePoints[1].x,
                      intersection.intersectedTriangle.texturePoints[1].y);
  glm::vec2 v2Texture(intersection.intersectedTriangle.texturePoints[2].x,
                      intersection.intersectedTriangle.texturePoints[2].y);
  ModelTriangle triangle = intersection.intersectedTriangle;
  glm::vec3 point = intersection.intersectionPoint;
  float barycoord[3];
  glm::vec3 v1v0 = triangle.vertices[0] - triangle.vertices[1];
  glm::vec3 v1v2 = triangle.vertices[2] - triangle.vertices[1];
  glm::vec3 v1point = point - triangle.vertices[1];
  float d00 = glm::dot(v1v0, v1v0);
  float d01 = glm::dot(v1v0, v1v2);
  float d11 = glm::dot(v1v2, v1v2);
  float denominator = d00 * d11 - d01 * d01;
  float d20 = glm::dot(v1point, v1v0);
  float d21 = glm::dot(v1point, v1v2);
  barycoord[0] = (d11 * d20 - d01 * d21) / denominator;
  barycoord[1] = (d00 * d21 - d01 * d20) / denominator;
  barycoord[2] = 1.0f - barycoord[0] - barycoord[1];
  pointNormal = barycoord[0] * v0Normal + barycoord[2] * v1Normal +
                barycoord[1] * v2Normal;
  pointTexture = barycoord[0] * v0Texture + barycoord[2] * v1Texture +
                 barycoord[1] * v2Texture;
}

glm::vec3 refract(const glm::vec3 rayDirection, const glm::vec3 pointNormal,
                  float ior) {
  float help = glm::dot(-rayDirection, pointNormal);
  float cos_theta = 1.0f;
  if (help < 1) {
    cos_theta = help;
  }
  glm::vec3 r_out_perp = ior * (rayDirection + cos_theta * pointNormal);
  glm::vec3 r_out_parallel =
      float(-std::sqrt(std::fabs(1.0 - glm::dot(r_out_perp, r_out_perp)))) *
      pointNormal;
  return r_out_perp + r_out_parallel;
}

Colour treatReflectionAndRefraction(
    bool insideRefracted, int depth, RayTriangleIntersection intersection,
    glm::vec3 rayDirection, glm::vec3 pointNormal, glm::vec2 pointTexture,
    std::vector<ModelTriangle> &modelTriangle,
    std::vector<glm::vec3> &verticesNormals,
    std::vector<std::vector<int>> &trianglesVerticesIndexes,
    TextureMap &textureMap) {
  if (intersection.intersectedTriangle.colour.name == "Red" &&
      depth <= MAX_DEPTH) {
    glm::vec3 refractedRay(0, 0, 0);
    if (!insideRefracted) {
      refractedRay = refract(rayDirection, pointNormal, 1.5f);
      insideRefracted = true;
    } else {
      pointNormal = -1.f * pointNormal;
      refractedRay = refract(rayDirection, pointNormal, 1 / 1.5f);
      insideRefracted = false;
    }

    refractedRay = glm::normalize(refractedRay);

    RayTriangleIntersection refractionIntersection;
    int trianglesRefractionIntersected = 0;
    bool foundRefractionIntersection = getClosestIntersection(
        intersection.intersectionPoint - pointNormal * 0.1f, refractedRay,
        modelTriangle, refractionIntersection, trianglesRefractionIntersected);

    if (foundRefractionIntersection) {
      glm::vec3 pointRefractionNormal;
      glm::vec2 pointRefractionTexture;
      barycentricInterpolation(refractionIntersection, verticesNormals,
                               trianglesVerticesIndexes, pointRefractionNormal,
                               pointRefractionTexture);
      pointRefractionNormal = glm::normalize(pointRefractionNormal);
      Colour refractedColour = treatReflectionAndRefraction(
          insideRefracted, depth + 1, refractionIntersection,
          refractionIntersection.intersectionPoint, pointRefractionNormal,
          pointRefractionTexture, modelTriangle, verticesNormals,
          trianglesVerticesIndexes, textureMap);
      refractedColour.blue +=
          0.1 * intersection.intersectedTriangle.colour.blue;
      refractedColour.red += 0.1 * intersection.intersectedTriangle.colour.red;
      refractedColour.green +=
          0.1 * intersection.intersectedTriangle.colour.green;
      return refractedColour;
    } else {
      return Colour(0, 0, 0);
    }
  }
  if ((intersection.intersectedTriangle.colour.name == "Blue") &&
      depth <= MAX_DEPTH) {
    glm::vec3 Ri = rayDirection;
    glm::vec3 Rr =
        glm::normalize(Ri - 2.0f * pointNormal * glm::dot(pointNormal, Ri));
    RayTriangleIntersection reflectionIntersection;
    int trianglesReflectionIntersected = 0;

    bool foundReflectionIntersection = getClosestIntersection(
        intersection.intersectionPoint + pointNormal * 0.001f, Rr,
        modelTriangle, reflectionIntersection, trianglesReflectionIntersected);
    if (foundReflectionIntersection) {
      glm::vec3 pointReflectionNormal;
      glm::vec2 pointReflectionTexture;
      barycentricInterpolation(reflectionIntersection, verticesNormals,
                               trianglesVerticesIndexes, pointReflectionNormal,
                               pointReflectionTexture);

      pointReflectionNormal = glm::normalize(pointReflectionNormal);
      Colour reflectionColour = treatReflectionAndRefraction(
          insideRefracted, depth + 1, reflectionIntersection,
          reflectionIntersection.intersectionPoint, pointReflectionNormal,
          pointReflectionTexture, modelTriangle, verticesNormals,
          trianglesVerticesIndexes, textureMap);
      return reflectionColour;
    } else {
      return Colour(0, 0, 0);
    }
  }
  if (IsTextured(intersection.intersectedTriangle)) {
    pointTexture.x = int(pointTexture.x) % textureMap.width;
    pointTexture.y = int(pointTexture.y) % textureMap.height;
    uint32_t pixelColour =
        textureMap.pixels[pointTexture.y * textureMap.width + pointTexture.x];
    return Colour((pixelColour >> 16) & 0xff, (pixelColour >> 8) & 0xff,
                  pixelColour & 0xff);
  }
  return intersection.intersectedTriangle.colour;
}

void drawRayTracedScene(DrawingWindow &window, glm::vec3 cameraPosition,
                        glm::mat3 cameraOrientation, float focalPoint,
                        std::vector<ModelTriangle> modelTriangle,
                        std::vector<glm::vec3> &verticesNormals,
                        std::vector<glm::vec2> &verticesTexture,
                        std::vector<std::vector<int>> &trianglesVerticesIndexes,
                        std::vector<glm::vec3> lightSources,
                        TextureMap textureMap) {
  window.clearPixels();
  for (size_t y = 0; y < window.height; y++) {
    for (size_t x = 0; x < window.width; x++) {
      glm::vec3 rayDirection = getSpatialIntersectionPoint(
          cameraPosition, cameraOrientation, glm::vec3(x, y, 0), focalPoint);
      RayTriangleIntersection closestIntersection;
      int trianglesIntersected = 0;
      bool foundIntersection =
          getClosestIntersection(cameraPosition, rayDirection, modelTriangle,
                                 closestIntersection, trianglesIntersected);
      if (foundIntersection) {
        glm::vec3 pointNormal;
        glm::vec2 pointTexture;
        barycentricInterpolation(closestIntersection, verticesNormals,
                                 trianglesVerticesIndexes, pointNormal,
                                 pointTexture);

        pointNormal = glm::normalize(pointNormal);
        pointTexture.x = int(pointTexture.x) % textureMap.width;
        pointTexture.y = int(pointTexture.y) % textureMap.height;
        uint32_t pixelColour =
            textureMap
                .pixels[pointTexture.y * textureMap.width + pointTexture.x];
        Colour currentColour = treatReflectionAndRefraction(
            false, 0, closestIntersection, rayDirection, pointNormal,
            pointTexture, modelTriangle, verticesNormals,
            trianglesVerticesIndexes, textureMap);

        float diffuse =
            diffuseLight(lightSources, modelTriangle, closestIntersection,
                         cameraPosition, pointNormal);
        float specular = specularLight(cameraPosition, lightSources,
                                       closestIntersection, pointNormal);
        float ambient = 0.2f;
        float brightness = diffuse + specular + ambient;
        uint32_t pixel = 0;
        if (IsTextured(closestIntersection.intersectedTriangle)) {
          pixel = getPixel(brightness, pixelColour);
        } else {
          pixel = getPixel(brightness, currentColour);
        }

        window.setPixelColour(x, y, pixel);
      }
    }
  }
}

void draw_triangle(DrawingWindow &window, CanvasTriangle triangle,
                   Colour colour,
                   std::vector<std::vector<float>> &depthBuffer) {
  draw_line(window, triangle.v0(), triangle.v1(), colour, depthBuffer);
  draw_line(window, triangle.v1(), triangle.v2(), colour, depthBuffer);
  draw_line(window, triangle.v2(), triangle.v0(), colour, depthBuffer);
}
void drawWireframed(DrawingWindow &window, glm::vec3 cameraPosition,
                    glm::mat3 cameraOrientation, float focalPoint,
                    std::vector<ModelTriangle> modelTriangle,
                    std::vector<std::vector<float>> &depthBuffer) {
  for (auto &it : depthBuffer) {
    std::fill(it.begin(), it.end(), 0);
  }
  window.clearPixels();
  for (ModelTriangle &it : modelTriangle) {
    CanvasPoint first_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[0], it.texturePoints[0],
        focalPoint);
    CanvasPoint second_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[1], it.texturePoints[1],
        focalPoint);
    CanvasPoint third_vertex = getCanvasIntersectionPoint(
        cameraPosition, cameraOrientation, it.vertices[2], it.texturePoints[2],
        focalPoint);
    CanvasTriangle triangle(first_vertex, second_vertex, third_vertex);
    Colour triangleColour(it.colour.red, it.colour.green, it.colour.blue);
    draw_triangle(window, triangle, triangleColour, depthBuffer);
  }
}
void printMat3(glm::mat3 mat) {
  printVec3(mat[0]);
  printVec3(mat[1]);
  printVec3(mat[2]);
  std::cout << std::endl;
}
void computeVerticesNormals(std::vector<glm::vec3> &verticesNormals) {
  for (auto &v : verticesNormals) {
    v = glm::normalize(v);
  }
}
int main(int argc, char *argv[]) {
  std::unordered_map<std::string, Colour> colourMap;
  TextureMap textureMap;
  if (!parseMTL("src/textured-cornell-box.mtl", colourMap, textureMap)) {
    std::cout << "ERROR\n";
    return 0;
  };
  std::cout << textureMap.height << ' ' << textureMap.width
            << textureMap.pixels[2] << std::endl;
  std::vector<ModelTriangle> modelTriangle;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> verticesNormals;
  std::vector<glm::vec2> verticesTexture;
  std::vector<std::vector<int>> trianglesVerticesIndexes;
  if (!parseOBJ("src/textured-cornell-box.obj", modelTriangle, vertices,
                verticesNormals, verticesTexture, trianglesVerticesIndexes,
                colourMap, textureMap, 0.17f)) {
    std::cout << "ERROR\n";
    return 0;
  };
  computeVerticesNormals(verticesNormals);
  glm::vec3 cameraPosition(0.0, 0.0, 3.0);
  float focalPoint = 2.0;
  std::vector<std::vector<float>> depthBuffer(WIDTH,
                                              std::vector<float>(HEIGHT, 0.0));
  glm::mat3 cameraOrientation(1.0f);
  glm::vec3 origin(0.0f, 0.0f, 0.0f);

  std::vector<glm::vec3> lightSources{
      glm::vec3(0.0f, 0.3f, 0.0f),    glm::vec3(0.015f, 0.3f, 0.0f),
      glm::vec3(-0.015f, 0.3f, 0.0f), glm::vec3(0.0f, 0.3f, 0.015f),
      glm::vec3(0.0f, 0.3f, -0.015f), glm::vec3(0.0f, 0.3f, 0.0f),
      glm::vec3(0.035f, 0.3f, 0.0f),  glm::vec3(-0.035f, 0.3f, 0.0f),
      glm::vec3(0.0f, 0.3f, 0.035f),  glm::vec3(0.0f, 0.3f, -0.035f),
      glm::vec3(0.05f, 0.3f, 0.0f),   glm::vec3(-0.05f, 0.3f, 0.0f),
      glm::vec3(0.0f, 0.3f, 0.05f),   glm::vec3(0.0f, 0.3f, -0.05f),
      glm::vec3(0.065f, 0.3f, 0.0f),  glm::vec3(-0.065f, 0.3f, 0.0f),
      glm::vec3(0.0f, 0.3f, 0.065f),  glm::vec3(0.0f, 0.3f, -0.065f),
  };
  DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
  SDL_Event event;
  Render renderer = WIREFRAME;
  float frontWalk = 0;
  float totalAngle = 0;
  //    while (true) {
  for (int i = 0; i < 1000; ++i) {
    std::cout << i << std::endl;
    if (window.pollForInputEvents(event))
      handleEvent(event, window, cameraPosition, cameraOrientation, depthBuffer,
                  renderer, lightSources);
    if (i < 120) {
      drawWireframed(window, cameraPosition, cameraOrientation, focalPoint,
                     modelTriangle, depthBuffer);
      orbit(cameraPosition, cameraOrientation, totalAngle, frontWalk, 1.f);
      glm::mat3 view = lookAt(cameraPosition, origin);
      cameraOrientation = view;
    } else if (i >= 120 && i < 240) {
      drawRasterisedScene(window, cameraPosition, cameraOrientation, focalPoint,
                          modelTriangle, depthBuffer, textureMap);
      orbit(cameraPosition, cameraOrientation, totalAngle, frontWalk, 1.f);
      glm::mat3 view = lookAt(cameraPosition, origin);
      cameraOrientation = view;
    } else if (i >= 260) {
      frontWalk *= -1;
      drawRayTracedScene(window, cameraPosition, cameraOrientation, focalPoint,
                         modelTriangle, verticesNormals, verticesTexture,
                         trianglesVerticesIndexes, lightSources, textureMap);
      orbit(cameraPosition, cameraOrientation, totalAngle, frontWalk, 1.f);
      glm::mat3 view = lookAt(cameraPosition, origin);
      cameraOrientation = view;
    }
    window.renderFrame();
    //   std::string file = "animation2/" + std::to_string(i) + ".ppm";
    //   window.savePPM(file);
  }
}
