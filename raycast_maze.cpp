// Compile with:
// For SDL2 (default):
//   g++ -std=c++17 -O3 -Wall -DUSE_BOTH raycast_maze.cpp -o raycast_maze $(sdl2-config --cflags --libs)
// For SDL1 (define USE_SDL1):
//   g++ -std=c++17 -O3 -Wall -DUSE_SDL1 -DUSE_BOTH raycast_maze.cpp -o raycast_maze -lSDL
// ((-D)USE_BOTH enables both finish and exploration-threshold logic. Do not define both USE_THRESHOLD and USE_FINISH separately.)

#if defined(USE_THRESHOLD) && defined(USE_FINISH)
  #error "Both USE_THRESHOLD and USE_FINISH are defined! Use USE_BOTH to enable both."
#endif
// ----------------------------------------------------------------------
// Include SDL headers based on SDL version
// ----------------------------------------------------------------------
#ifndef USE_SDL1
  #include <SDL2/SDL.h>
  // SDL2-specific compatibility macros:
  #define SET_HINT(key, value) SDL_SetHint(key, value)
  #define WINDOW_SHOWN SDL_WINDOW_SHOWN
  #define WINDOW_FULLSCREEN SDL_WINDOW_FULLSCREEN
  #define DESKTOP_DISPLAY_MODE SDL_DisplayMode
  #define GET_DESKTOP_DISPLAY_MODE(i, dm) SDL_GetDesktopDisplayMode(i, dm)
  #define CREATE_WINDOW(title, x, y, w, h, flags) SDL_CreateWindow(title, x, y, w, h, flags)
  #define GET_WINDOW_SURFACE(win) SDL_GetWindowSurface(win)
  #define UPDATE_WINDOW_SURFACE(win) SDL_UpdateWindowSurface(win)
  #define DESTROY_WINDOW(win) SDL_DestroyWindow(win)
  #define SET_RELATIVE_MOUSE_MODE(mode) SDL_SetRelativeMouseMode(mode)
#else
  #include <SDL/SDL.h>
  #include <SDL/SDL_keyboard.h>
  // SDL1: define dummy macros for SDL2-only functions.
  #define SET_HINT(key, value)
  #define WINDOW_SHOWN 0
  #define WINDOW_FULLSCREEN 0
  // (SDL1 uses SDL_SetVideoMode; no window functions exist.)
#endif

#include "wallbmp.h"  // Must define wall_bmp and wall_bmp_len for a valid 64x64 BMP.
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
#include <vector>

// ----------------------------------------------------------------------
// Constants and Global Variables
// ----------------------------------------------------------------------
constexpr int MAP_WIDTH  = 250;
constexpr int MAP_HEIGHT = 250;
constexpr int TEX_WIDTH  = 64;
constexpr int TEX_HEIGHT = 64;
constexpr int defaultScreenWidth  = 1920;
constexpr int defaultScreenHeight = 1080;
constexpr int explorationThreshold = 300;  // Only used if USE_THRESHOLD or USE_BOTH is defined.
constexpr Uint32 levelResetDelay = 5000;     // (Not used here.)

// The game map: 0 = empty, 1 = wall, 2 = finish.
int gameMap[MAP_HEIGHT][MAP_WIDTH];

// Exploration tracking.
bool visitedCells[MAP_HEIGHT][MAP_WIDTH] = { false };
int distinctVisited = 0;

// ----------------------------------------------------------------------
// Maze Generation and Exploration Tracking Functions
// ----------------------------------------------------------------------
void initVisited() {
    for (int y = 0; y < MAP_HEIGHT; ++y)
        for (int x = 0; x < MAP_WIDTH; ++x)
            visitedCells[y][x] = false;
    distinctVisited = 0;
}

void updateVisited(double posX, double posY) {
    int cellX = static_cast<int>(posX);
    int cellY = static_cast<int>(posY);
    if (cellX >= 0 && cellX < MAP_WIDTH &&
        cellY >= 0 && cellY < MAP_HEIGHT &&
        !visitedCells[cellY][cellX])
    {
        visitedCells[cellY][cellX] = true;
        ++distinctVisited;
    }
}

// Place a finish cell (value 2) randomly along one border and open an adjacent passage.
void placeRandomFinish() {
    std::random_device rd;
    std::mt19937 gen(rd());
    // 0 = TOP, 1 = BOTTOM, 2 = LEFT, 3 = RIGHT.
    std::uniform_int_distribution<> borderDist(0, 3);
    int border = borderDist(gen);
    int fX, fY;
    if (border == 0) { // TOP
        fY = 0;
        std::uniform_int_distribution<> colDist(1, MAP_WIDTH - 2);
        fX = colDist(gen);
        gameMap[fY + 1][fX] = 0;  // open passage below finish cell
    } else if (border == 1) { // BOTTOM
        fY = MAP_HEIGHT - 1;
        std::uniform_int_distribution<> colDist(1, MAP_WIDTH - 2);
        fX = colDist(gen);
        gameMap[fY - 1][fX] = 0;
    } else if (border == 2) { // LEFT
        fX = 0;
        std::uniform_int_distribution<> rowDist(1, MAP_HEIGHT - 2);
        fY = rowDist(gen);
        gameMap[fY][fX + 1] = 0;
    } else { // RIGHT
        fX = MAP_WIDTH - 1;
        std::uniform_int_distribution<> rowDist(1, MAP_HEIGHT - 2);
        fY = rowDist(gen);
        gameMap[fY][fX - 1] = 0;
    }
    gameMap[fY][fX] = 2;
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
        std::vector<std::pair<int,int>> neighbors;
        for (auto [dx, dy] : directions) {
            int nx = cx + dx, ny = cy + dy;
            if (nx > 0 && nx < MAP_WIDTH - 1 &&
                ny > 0 && ny < MAP_HEIGHT - 1 &&
                gameMap[ny][nx] == 1)
            {
                neighbors.push_back({nx, ny});
            }
        }
        if (!neighbors.empty()) {
            std::uniform_int_distribution<> dis(0, (int)neighbors.size()-1);
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
    for (int y = 0; y < MAP_HEIGHT; ++y) {
        gameMap[y][0] = 1;
        gameMap[y][MAP_WIDTH-1] = 1;
    }
    for (int x = 0; x < MAP_WIDTH; ++x) {
        gameMap[0][x] = 1;
        gameMap[MAP_HEIGHT-1][x] = 1;
    }
#if !defined(USE_THRESHOLD) || defined(USE_FINISH) || defined(USE_BOTH)
    placeRandomFinish();
#endif
}

// ----------------------------------------------------------------------
// Accessor and Finish Condition
// ----------------------------------------------------------------------
inline int getCell(int x, int y) {
    return (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) ? 1 : gameMap[y][x];
}

bool checkWin(double posX, double posY) {
    return (getCell(static_cast<int>(posX), static_cast<int>(posY)) == 2);
}

// ----------------------------------------------------------------------
// Collision Resolution (Circle-Based)
// ----------------------------------------------------------------------
// The player is represented as a circle (radius = hitbox).
// We treat cell 1 as a wall and cells 0 and 2 (finish) as passable.
void resolveCircleCollision(double &posX, double &posY, double hitbox) {
    int startX = std::max(0, static_cast<int>(std::floor(posX - hitbox)));
    int endX   = std::min(MAP_WIDTH - 1, static_cast<int>(std::ceil(posX + hitbox)));
    int startY = std::max(0, static_cast<int>(std::floor(posY - hitbox)));
    int endY   = std::min(MAP_HEIGHT - 1, static_cast<int>(std::ceil(posY + hitbox)));
    
    for (int cy = startY; cy <= endY; ++cy) {
        for (int cx = startX; cx <= endX; ++cx) {
            if (getCell(cx, cy) == 1) {
                double closestX = std::max((double)cx, std::min(posX, (double)(cx+1)));
                double closestY = std::max((double)cy, std::min(posY, (double)(cy+1)));
                double distX = posX - closestX;
                double distY = posY - closestY;
                double distSq = distX*distX + distY*distY;
                if (distSq < hitbox*hitbox) {
                    double dist = std::sqrt(distSq);
                    double pen = hitbox - dist;
                    if (dist == 0)
                        posY -= pen;
                    else {
                        posX += (distX/dist)*pen;
                        posY += (distY/dist)*pen;
                    }
                }
            }
        }
    }
}

void updatePlayerPosition(double &posX, double &posY, double dx, double dy, double hitbox) {
    double newX = posX + dx;
    double newY = posY + dy;
    resolveCircleCollision(newX, newY, hitbox);
    posX = newX;
    posY = newY;
}

// ----------------------------------------------------------------------
// Main Raycasting and Rendering
// ----------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // For SDL2 only: prevent minimizing on focus loss.
#ifndef USE_SDL1
    SET_HINT(SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS, "0");
#endif

    int screenWidth  = defaultScreenWidth;
    int screenHeight = defaultScreenHeight;
    // Use "-windowed" to run in windowed mode; default is fullscreen.
    bool fullscreen = true;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-windowed") {
            fullscreen = false;
            screenWidth  = 640;
            screenHeight = 480;
            break;
        }
    }
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    generateMaze();
    initVisited();
    
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }
    
    bool appFocused = true;
    
#ifndef USE_SDL1
    Uint32 windowFlags = WINDOW_SHOWN;
    if (fullscreen) {
        windowFlags |= WINDOW_FULLSCREEN;
        DESKTOP_DISPLAY_MODE dm;
        if (GET_DESKTOP_DISPLAY_MODE(0, &dm) == 0) {
            screenWidth  = dm.w;
            screenHeight = dm.h;
        } else {
            std::cerr << "SDL_GetDesktopDisplayMode failed: " << SDL_GetError() << "\n";
        }
    }
    SDL_Window* window = CREATE_WINDOW("Raycast Maze - Explore 300 Cells",
                                         SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                         screenWidth, screenHeight, windowFlags);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }
    SDL_Surface* screen = GET_WINDOW_SURFACE(window);
#else
    SDL_Surface* screen = SDL_SetVideoMode(screenWidth, screenHeight, 32, SDL_SWSURFACE);
    if (!screen) {
        std::cerr << "SDL_SetVideoMode failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }
    SDL_WM_SetCaption("Raycast Maze - Explore 300 Cells", nullptr);
#endif

    if (!screen) {
        std::cerr << "Failed to get screen surface\n";
#ifndef USE_SDL1
        DESTROY_WINDOW(window);
#endif
        SDL_Quit();
        return 1;
    }
    
    // Load wall texture.
    SDL_RWops* rw = SDL_RWFromConstMem(wall_bmp, wall_bmp_len);
    SDL_Surface* tempSurface = SDL_LoadBMP_RW(rw, 1);
    if (!tempSurface) {
        std::cerr << "SDL_LoadBMP_RW failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_Surface* wallTexture = SDL_ConvertSurface(tempSurface, screen->format, 0);
    SDL_FreeSurface(tempSurface);
    if (!wallTexture) {
#ifndef USE_SDL1
        DESTROY_WINDOW(window);
#endif
        std::cerr << "SDL_ConvertSurface failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }
    
    // Build a 2D array for texture sampling.
    Uint32 wallTexArr[TEX_HEIGHT][TEX_WIDTH];
    if (SDL_LockSurface(wallTexture) != 0) {
        std::cerr << "SDL_LockSurface failed: " << SDL_GetError() << "\n";
#ifndef USE_SDL1
        DESTROY_WINDOW(window);
#endif
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
    
#ifndef USE_SDL1
    SET_RELATIVE_MOUSE_MODE(SDL_TRUE);
#endif
    
    // Player and camera settings.
    double posX_player = 1.5, posY_player = 1.5;
    double dirX = 1.0, dirY = 0.0;
    double planeX = 0.0, planeY = (fullscreen ? 1.0 : 0.66);
    const double hitbox = 0.2;
    
    // Main loop.
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
#ifndef USE_SDL1
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)
                    appFocused = true;
                else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
                    appFocused = false;
            }
#else
            // SDL1 does not have window events.
#endif
            if (event.type == SDL_QUIT)
                quitApp = true;
#ifndef USE_SDL1
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
                quitApp = true;
#else
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
                quitApp = true;
#endif
#ifndef USE_SDL1
            if (event.type == SDL_MOUSEMOTION) {
                double rot = event.motion.xrel * mouseSensitivity;
                double oldDirX = dirX;
                dirX = dirX * cos(rot) - dirY * sin(rot);
                dirY = oldDirX * sin(rot) + dirY * cos(rot);
                double oldPlaneX = planeX;
                planeX = planeX * cos(rot) - planeY * sin(rot);
                planeY = oldPlaneX * sin(rot) + planeY * cos(rot);
            }
#else
            if (event.type == SDL_MOUSEMOTION) {
                double rot = event.motion.xrel * mouseSensitivity;
                double oldDirX = dirX;
                dirX = dirX * cos(rot) - dirY * sin(rot);
                dirY = oldDirX * sin(rot) + dirY * cos(rot);
                double oldPlaneX = planeX;
                planeX = planeX * cos(rot) - planeY * sin(rot);
                planeY = oldPlaneX * sin(rot) + planeY * cos(rot);
            }
#endif
        }
        
        if (appFocused) {
#ifndef USE_SDL1
            double moveSpeed = 3.0 * frameTime;
            const Uint8* keys = SDL_GetKeyboardState(NULL);
            // Support for arrow keys turning (SDL2)
            if (keys[SDL_SCANCODE_RIGHT]) {
                double rotSpeed = 2.0 * frameTime;
                double oldDirX = dirX;
                dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
                dirY = oldDirX * sin(rotSpeed) + dirY * cos(rotSpeed);
                double oldPlaneX = planeX;
                planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
                planeY = oldPlaneX * sin(rotSpeed) + planeY * cos(rotSpeed);
            }
            if (keys[SDL_SCANCODE_LEFT]) {
                double rotSpeed = 2.0 * frameTime;
                double oldDirX = dirX;
                dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
                dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
                double oldPlaneX = planeX;
                planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
                planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
            }
            if (keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT])
                moveSpeed *= 2.0;
            double dx = 0, dy = 0;
            if (keys[SDL_SCANCODE_W]) { dx += dirX * moveSpeed; dy += dirY * moveSpeed; }
            if (keys[SDL_SCANCODE_S]) { dx -= dirX * moveSpeed; dy -= dirY * moveSpeed; }
            if (keys[SDL_SCANCODE_A]) { dx += dirY * moveSpeed; dy -= dirX * moveSpeed; }
            if (keys[SDL_SCANCODE_D]) { dx -= dirY * moveSpeed; dy += dirX * moveSpeed; }
#else
            int numKeys;
            #ifndef SDL_GetKeyState
            extern DECLSPEC Uint8 * SDLCALL SDL_GetKeyState(int *numkeys);
            #endif
            const Uint8* keys = SDL_GetKeyState(&numKeys);
            double moveSpeed = 3.0 * frameTime;
            // Support for arrow keys turning (SDL1)
            if (keys[SDLK_LEFT]) {
                double rotSpeed = 2.0 * frameTime;
                double oldDirX = dirX;
                dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
                dirY = oldDirX * sin(rotSpeed) + dirY * cos(rotSpeed);
                double oldPlaneX = planeX;
                planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
                planeY = oldPlaneX * sin(rotSpeed) + planeY * cos(rotSpeed);
            }
            if (keys[SDLK_RIGHT]) {
                double rotSpeed = 2.0 * frameTime;
                double oldDirX = dirX;
                dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
                dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
                double oldPlaneX = planeX;
                planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
                planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
            }
            if (keys[SDLK_LSHIFT] || keys[SDLK_RSHIFT])
                moveSpeed *= 2.0;
            double dx = 0, dy = 0;
            if (keys[SDLK_w]) { dx += dirX * moveSpeed; dy += dirY * moveSpeed; }
            if (keys[SDLK_s]) { dx -= dirX * moveSpeed; dy -= dirY * moveSpeed; }
            if (keys[SDLK_a]) { dx += dirY * moveSpeed; dy -= dirX * moveSpeed; }
            if (keys[SDLK_d]) { dx -= dirY * moveSpeed; dy += dirX * moveSpeed; }
#endif
            updatePlayerPosition(posX_player, posY_player, dx, dy, hitbox);
            
#ifdef USE_THRESHOLD
            updateVisited(posX_player, posY_player);
            if (distinctVisited >= explorationThreshold) {
#ifndef USE_SDL1
                SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,
                    "Level Complete", "You explored long enough, new level! Click OK to continue.", window);
#else
                std::cout << "Level Complete: You explored long enough, new level! Press any key to continue." << std::endl;
                SDL_PollEvent(nullptr);
#endif
                generateMaze();
                posX_player = 1.5; posY_player = 1.5;
                dirX = 1.0; dirY = 0.0;
                planeX = 0.0; planeY = (fullscreen ? 1.0 : 0.66);
                initVisited();
            }
#endif

#if defined(USE_FINISH) || defined(USE_BOTH) || !defined(USE_THRESHOLD)
            if (checkWin(posX_player, posY_player)) {
#ifndef USE_SDL1
                SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,
                    "Level Complete", "You found the finish! Click OK to generate a new maze.", window);
#else
                std::cout << "Level Complete: You found the finish! Press any key to continue." << std::endl;
                SDL_PollEvent(nullptr);
#endif
                generateMaze();
                posX_player = 1.5; posY_player = 1.5;
                dirX = 1.0; dirY = 0.0;
                planeX = 0.0; planeY = (fullscreen ? 1.0 : 0.66);
                initVisited();
            }
#endif
        }
        
#ifndef USE_SDL1
        SDL_Rect ceilingRect = {0, 0, screenWidth, screenHeight / 2};
        SDL_FillRect(screen, &ceilingRect, SDL_MapRGB(screen->format, 70, 70, 70));
        SDL_Rect floorRect = {0, screenHeight / 2, screenWidth, screenHeight / 2};
        SDL_FillRect(screen, &floorRect, SDL_MapRGB(screen->format, 40, 40, 40));
#else
        SDL_FillRect(screen, nullptr, SDL_MapRGB(screen->format, 70,70,70));
#endif
        
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
            
            int hit = 0, side = 0;
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
            if (drawStart < 0)
                drawStart = 0;
            int drawEnd = lineHeight / 2 + screenHeight / 2;
            if (drawEnd >= screenHeight)
                drawEnd = screenHeight - 1;
            
            double wallX = (side == 0)
                ? posY_player + perpWallDist * rayDirY
                : posX_player + perpWallDist * rayDirX;
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
                    // Checkerboard pattern for finish cell.
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
        
#ifndef USE_SDL1
        SDL_UnlockSurface(screen);
        UPDATE_WINDOW_SURFACE(window);
#else
        SDL_Flip(screen);
#endif
    }
    
#ifndef USE_SDL1
    DESTROY_WINDOW(window);
#endif
    SDL_FreeSurface(wallTexture);
    SDL_Quit();
    return 0;
}
