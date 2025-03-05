// Compile with:
// g++ -std=c++17 -O3 -Wall raycast_maze.cpp -o raycast_maze $(sdl2-config --cflags --libs)

#include "wallbmp.h"  // Must define wall_bmp and wall_bmp_len for a valid 64x64 BMP.
#include <SDL2/SDL.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random>
#include <stack>
#include <queue>
#include <utility>
#include <algorithm>
#include <string>

// Fixed map dimensions.
constexpr int MAP_WIDTH = 250;
constexpr int MAP_HEIGHT = 250;

// Finish cell (placed on odd indices).
constexpr int finishX = MAP_WIDTH - 3;
constexpr int finishY = MAP_HEIGHT - 3;

// Texture dimensions.
constexpr int TEX_WIDTH = 64;
constexpr int TEX_HEIGHT = 64;

// Default screen dimensions.
constexpr int defaultScreenWidth = 640;
constexpr int defaultScreenHeight = 480;

// Exploration threshold: if the player visits 300 distinct cells.
constexpr int explorationThreshold = 300;

// The game map: 0 = empty, 1 = wall, 2 = finish.
int gameMap[MAP_HEIGHT][MAP_WIDTH];

// Exploration tracking.
bool visitedCells[MAP_HEIGHT][MAP_WIDTH] = { false };
int distinctVisited = 0;

void initVisited() {
    for (int y = 0; y < MAP_HEIGHT; y++)
        for (int x = 0; x < MAP_WIDTH; x++)
            visitedCells[y][x] = false;
    distinctVisited = 0;
}

void updateVisited(double posX, double posY) {
    int cellX = static_cast<int>(posX);
    int cellY = static_cast<int>(posY);
    if (!visitedCells[cellY][cellX]) {
        visitedCells[cellY][cellX] = true;
        ++distinctVisited;
    }
}

// Generate the maze via recursive backtracking.
void generateMaze() {
    for (int y = 0; y < MAP_HEIGHT; ++y)
        for (int x = 0; x < MAP_WIDTH; ++x)
            gameMap[y][x] = 1;
    gameMap[1][1] = 0;
    std::stack<std::pair<int,int>> stack;
    stack.push({1, 1});
    
    std::random_device rd;
    std::mt19937 gen(rd());
    const std::pair<int,int> directions[4] = { {0,-2}, {0,2}, {-2,0}, {2,0} };
    
    while (!stack.empty()) {
        auto [cx, cy] = stack.top();
        std::pair<int,int> neighbors[4];
        int nCount = 0;
        for (const auto& [dx, dy] : directions) {
            int nx = cx + dx, ny = cy + dy;
            if (nx > 0 && nx < MAP_WIDTH - 1 && ny > 0 && ny < MAP_HEIGHT - 1 && gameMap[ny][nx] == 1)
                neighbors[nCount++] = {nx, ny};
        }
        if (nCount > 0) {
            std::uniform_int_distribution<> dis(0, nCount - 1);
            auto [nx, ny] = neighbors[dis(gen)];
            int wallX = cx + (nx - cx) / 2;
            int wallY = cy + (ny - cy) / 2;
            gameMap[wallY][wallX] = 0;
            gameMap[ny][nx] = 0;
            stack.push({nx, ny});
        } else {
            stack.pop();
        }
    }
    // Force borders to be walls.
    for (int y = 0; y < MAP_HEIGHT; y++) {
        gameMap[y][0] = 1;
        gameMap[y][MAP_WIDTH - 1] = 1;
    }
    for (int x = 0; x < MAP_WIDTH; x++) {
        gameMap[0][x] = 1;
        gameMap[MAP_HEIGHT - 1][x] = 1;
    }
    // Mark finish cell.
    gameMap[finishY][finishX] = 2;
}

// Inline accessor.
inline int getCell(int x, int y) {
    return (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) ? 1 : gameMap[y][x];
}

// Check win condition.
bool checkWin(double posX, double posY) {
    return (getCell(static_cast<int>(posX), static_cast<int>(posY)) == 2);
}

// -------------------------------
// Collision Resolution (Circle-Based)
// -------------------------------
// The player is represented as a circle (radius = hitbox).
// This function pushes the player out of any overlapping wall cell.
void resolveCircleCollision(double &posX, double &posY, double hitbox) {
    int startX = std::max(0, static_cast<int>(std::floor(posX - hitbox)));
    int endX   = std::min(MAP_WIDTH - 1, static_cast<int>(std::ceil(posX + hitbox)));
    int startY = std::max(0, static_cast<int>(std::floor(posY - hitbox)));
    int endY   = std::min(MAP_HEIGHT - 1, static_cast<int>(std::ceil(posY + hitbox)));
    
    for (int cy = startY; cy <= endY; ++cy) {
        for (int cx = startX; cx <= endX; ++cx) {
            if (getCell(cx, cy) != 0) {
                double closestX = std::max((double)cx, std::min(posX, (double)(cx + 1)));
                double closestY = std::max((double)cy, std::min(posY, (double)(cy + 1)));
                double distX = posX - closestX;
                double distY = posY - closestY;
                double distSq = distX * distX + distY * distY;
                if (distSq < hitbox * hitbox) {
                    double dist = std::sqrt(distSq);
                    double penetration = hitbox - dist;
                    if (dist == 0) {
                        posY -= penetration;
                    } else {
                        posX += (distX / dist) * penetration;
                        posY += (distY / dist) * penetration;
                    }
                }
            }
        }
    }
}

// Update player position based on input and resolve collisions.
void updatePlayerPosition(double &posX, double &posY, double dx, double dy, double hitbox) {
    double newX = posX + dx;
    double newY = posY + dy;
    resolveCircleCollision(newX, newY, hitbox);
    posX = newX;
    posY = newY;
}

// -------------------------------
// Main Raycasting and Rendering
// -------------------------------

int main(int argc, char* argv[]) {
    // Prevent SDL from minimizing on focus loss.
    SDL_SetHint(SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS, "0");
    
    // Process command-line arguments: only "-fullscreen" is supported.
    bool fullscreen = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-fullscreen") {
            fullscreen = true;
            break;
        }
    }
    
    int screenWidth = defaultScreenWidth;
    int screenHeight = defaultScreenHeight;
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    generateMaze();
    initVisited();
    
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }
    
    // Track app focus.
    bool appFocused = true;
    
    Uint32 windowFlags = SDL_WINDOW_SHOWN;
    if (fullscreen) {
        windowFlags |= SDL_WINDOW_FULLSCREEN;
        SDL_DisplayMode dm;
        if (SDL_GetDesktopDisplayMode(0, &dm) == 0) {
            screenWidth = dm.w;
            screenHeight = dm.h;
            std::cout << "Desktop resolution: " << screenWidth << "x" << screenHeight << std::endl;
        } else {
            std::cerr << "SDL_GetDesktopDisplayMode failed: " << SDL_GetError() << std::endl;
        }
    }
    
    SDL_Window* window = SDL_CreateWindow("Raycast Maze - Explore 300 Cells",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          screenWidth, screenHeight, windowFlags);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }
    
    SDL_Surface* screen = SDL_GetWindowSurface(window);
    if (!screen) {
        std::cerr << "Failed to get window surface: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    // Load wall texture from embedded BMP data.
    SDL_RWops* rw = SDL_RWFromConstMem(wall_bmp, wall_bmp_len);
    SDL_Surface* tempSurface = SDL_LoadBMP_RW(rw, 1);
    if (!tempSurface) {
        std::cerr << "SDL_LoadBMP_RW failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_Surface* wallTexture = SDL_ConvertSurface(tempSurface, screen->format, 0);
    SDL_FreeSurface(tempSurface);
    if (!wallTexture) {
        std::cerr << "SDL_ConvertSurface failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    // Build a fixed 2D array for texture sampling (wallTexArr).
    Uint32 wallTexArr[TEX_HEIGHT][TEX_WIDTH];
    if (SDL_LockSurface(wallTexture) != 0) {
        std::cerr << "SDL_LockSurface failed: " << SDL_GetError() << "\n";
        SDL_FreeSurface(wallTexture);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    Uint32* texPixels = static_cast<Uint32*>(wallTexture->pixels);
    int texPitch = wallTexture->pitch / sizeof(Uint32);
    for (int y = 0; y < TEX_HEIGHT; y++) {
        for (int x = 0; x < TEX_WIDTH; x++) {
            wallTexArr[y][x] = texPixels[y * texPitch + x];
        }
    }
    SDL_UnlockSurface(wallTexture);
    
    SDL_SetRelativeMouseMode(SDL_TRUE);
    
    // Player and camera settings.
    double posX_player = 1.5, posY_player = 1.5;
    double dirX = 1.0, dirY = 0.0;
    double planeX = 0.0, planeY = (fullscreen ? 1.0 : 0.66);
    const double hitbox = 0.2;
    
    // We'll use SDL_ShowSimpleMessageBox (blocking) so that when the message box appears,
    // the game waits for you to click "OK" before resetting.
    
    bool quitApp = false;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    const double mouseSensitivity = 0.003;
    
    while (!quitApp) {
        Uint32 currentTime = SDL_GetTicks();
        double frameTime = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;
        
        // Process events.
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)
                    appFocused = true;
                if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
                    appFocused = false;
            }
            if (event.type == SDL_QUIT)
                quitApp = true;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
                quitApp = true;
            if (event.type == SDL_MOUSEMOTION) {
                double rot = event.motion.xrel * mouseSensitivity;
                double oldDirX = dirX;
                dirX = dirX * cos(rot) - dirY * sin(rot);
                dirY = oldDirX * sin(rot) + dirY * cos(rot);
                double oldPlaneX = planeX;
                planeX = planeX * cos(rot) - planeY * sin(rot);
                planeY = oldPlaneX * sin(rot) + planeY * cos(rot);
            }
        }
        
        // Only update game logic if the app is focused.
        if (appFocused) {
            double moveSpeed = 3.0 * frameTime;
            // Shift-to-run: double speed if shift key is held.
            const Uint8* keystate = SDL_GetKeyboardState(NULL);
            if (keystate[SDL_SCANCODE_LSHIFT] || keystate[SDL_SCANCODE_RSHIFT])
                moveSpeed *= 2.0;
            double dx = 0, dy = 0;
            if (keystate[SDL_SCANCODE_W]) { dx += dirX * moveSpeed; dy += dirY * moveSpeed; }
            if (keystate[SDL_SCANCODE_S]) { dx -= dirX * moveSpeed; dy -= dirY * moveSpeed; }
            if (keystate[SDL_SCANCODE_A]) { dx += dirY * moveSpeed; dy -= dirX * moveSpeed; }
            if (keystate[SDL_SCANCODE_D]) { dx -= dirY * moveSpeed; dy += dirX * moveSpeed; }
            
            updatePlayerPosition(posX_player, posY_player, dx, dy, hitbox);
            
            updateVisited(posX_player, posY_player);
            if (distinctVisited >= explorationThreshold) {
                // When threshold reached, show message box. This call blocks until you click "OK".
                SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,
                    "Level Complete", "You explored long enough, new level!", window);
                generateMaze();
                posX_player = 1.5;
                posY_player = 1.5;
                dirX = 1.0; dirY = 0.0;
                planeX = 0.0; planeY = (fullscreen ? 1.0 : 0.66);
                initVisited();
            }
            
            if (checkWin(posX_player, posY_player)) {
                SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,
                    "Level Complete", "You beat the level! Generating new maze...", window);
                generateMaze();
                posX_player = 1.5;
                posY_player = 1.5;
                dirX = 1.0; dirY = 0.0;
                planeX = 0.0; planeY = (fullscreen ? 1.0 : 0.66);
                initVisited();
            }
        }
        
        // Draw ceiling and floor.
        SDL_Rect ceilingRect = {0, 0, screenWidth, screenHeight / 2};
        SDL_FillRect(screen, &ceilingRect, SDL_MapRGB(screen->format, 70, 70, 70));
        SDL_Rect floorRect = {0, screenHeight / 2, screenWidth, screenHeight / 2};
        SDL_FillRect(screen, &floorRect, SDL_MapRGB(screen->format, 40, 40, 40));
        
        if (SDL_LockSurface(screen) != 0) {
            std::cerr << "SDL_LockSurface failed: " << SDL_GetError() << "\n";
            break;
        }
        Uint32* screenPixels = static_cast<Uint32*>(screen->pixels);
        int screenPitch = screen->pitch / sizeof(Uint32);
        
        // Raycasting loop.
        for (int x = 0; x < screenWidth; x++) {
            double cameraX = 2.0 * x / screenWidth - 1.0;
            double rayDirX = dirX + planeX * cameraX;
            double rayDirY = dirY + planeY * cameraX;
            int mapX = static_cast<int>(posX_player), mapY = static_cast<int>(posY_player);
            double deltaDistX = (rayDirX == 0) ? 1e30 : std::abs(1.0 / rayDirX);
            double deltaDistY = (rayDirY == 0) ? 1e30 : std::abs(1.0 / rayDirY);
            int stepX = (rayDirX < 0) ? -1 : 1;
            int stepY = (rayDirY < 0) ? -1 : 1;
            double sideDistX = (rayDirX < 0) ? (posX_player - mapX) * deltaDistX : (mapX + 1.0 - posX_player) * deltaDistX;
            double sideDistY = (rayDirY < 0) ? (posY_player - mapY) * deltaDistY : (mapY + 1.0 - posY_player) * deltaDistY;
            
            int hit = 0, side;
            while (!hit) {
                if (sideDistX < sideDistY) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                }
                if (getCell(mapX, mapY) > 0)
                    hit = 1;
            }
            double perpWallDist = (side == 0)
                ? (mapX - posX_player + (1 - stepX) / 2.0) / rayDirX
                : (mapY - posY_player + (1 - stepY) / 2.0) / rayDirY;
            int lineHeight = static_cast<int>(screenHeight / perpWallDist);
            int drawStart = -lineHeight / 2 + screenHeight / 2;
            if (drawStart < 0) drawStart = 0;
            int drawEnd = lineHeight / 2 + screenHeight / 2;
            if (drawEnd >= screenHeight) drawEnd = screenHeight - 1;
            
            double wallX = (side == 0) ? posY_player + perpWallDist * rayDirY : posX_player + perpWallDist * rayDirX;
            wallX -= std::floor(wallX);
            int texX = static_cast<int>(wallX * TEX_WIDTH);
            if ((side == 0 && rayDirX > 0) || (side == 1 && rayDirY < 0))
                texX = TEX_WIDTH - texX - 1;
            int texStep = (TEX_HEIGHT << 16) / lineHeight;
            int texPos = (drawStart - screenHeight / 2 + lineHeight / 2) * texStep;
            
            int cellType = getCell(mapX, mapY);
            for (int y = drawStart; y < drawEnd; y++) {
                int texY = (texPos >> 16) & (TEX_HEIGHT - 1);
                texPos += texStep;
                Uint32 color;
                if (cellType == 2) {
                    int blockSize = 8;
                    color = (((texX / blockSize) + (texY / blockSize)) % 2 == 0)
                          ? SDL_MapRGB(screen->format, 255, 255, 255)
                          : SDL_MapRGB(screen->format, 0, 0, 0);
                } else {
                    color = wallTexArr[texY][texX];
                    if (side == 1) {
                        Uint8 a = (color >> 24) & 0xFF;
                        Uint8 r = ((color >> 16) & 0xFF) >> 1;
                        Uint8 g = ((color >> 8) & 0xFF) >> 1;
                        Uint8 b = (color & 0xFF) >> 1;
                        color = (a << 24) | (r << 16) | (g << 8) | b;
                    }
                }
                screenPixels[y * screenPitch + x] = color;
            }
        }
        
        SDL_UnlockSurface(screen);
        SDL_UpdateWindowSurface(window);
    }
    
    SDL_FreeSurface(wallTexture);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
