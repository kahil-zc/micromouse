#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#define MAZE_ACROSS 10
#define MAZE_AHEAD 5
#define GOAL_SIZE 2
#define MAZE_SIZE 10
#define START_X 0
#define START_Y 0
#include "maze28.inc"
// real maze: W x H, walls[x][y] bits NESW
int W,H; uint8_t R[10][10];
bool rin(int x,int y){return x>=0&&y>=0&&x<W&&y<H;}
void rset(int x,int y,int d){R[x][y]|=1<<d;int nx=x+DX[d],ny=y+DY[d];if(rin(nx,ny))R[nx][ny]|=1<<((d+2)&3);}
bool vis[10][10];
void carve(int x,int y){vis[x][y]=1;int o[4]={0,1,2,3};for(int i=3;i>0;i--){int j=rand()%(i+1);int t=o[i];o[i]=o[j];o[j]=t;}
 for(int i=0;i<4;i++){int d=o[i],nx=x+DX[d],ny=y+DY[d];if(rin(nx,ny)&&!vis[nx][ny]){R[x][y]&=~(1<<d);R[nx][ny]&=~(1<<((d+2)&3));carve(nx,ny);}}}
bool realWall(int x,int y,int d){int nx=x+DX[d],ny=y+DY[d]; if(!rin(nx,ny))return true; return R[x][y]&(1<<d);}
int main(){int fails=0,total=0;long steps=0;
 for(int t=0;t<2000;t++){srand(t); bool rot=t&1; W=rot?5:10;H=rot?10:5;
  memset(R,15,sizeof R);memset(vis,0,sizeof vis);carve(0,0);
  for(int k=0;k<W*H/8;k++){int x=rand()%W,y=rand()%H,d=rand()%4;int nx=x+DX[d],ny=y+DY[d];if(rin(nx,ny)){R[x][y]&=~(1<<d);R[nx][ny]&=~(1<<((d+2)&3));}}
  initMaze(); posX=0;posY=0;facing=0; setWall(0,0,2,true);
  bool toGoal=true; int n=0; bool ok=false;
  while(n<2000){ // search: sense
   setWall(posX,posY,facing,realWall(posX,posY,facing));
   setWall(posX,posY,(facing+1)&3,realWall(posX,posY,(facing+1)&3));
   setWall(posX,posY,(facing+3)&3,realWall(posX,posY,(facing+3)&3));
   flood(toGoal,false);
   if(dist[posX][posY]==255){break;}
   if(dist[posX][posY]==0){ if(toGoal){toGoal=false;continue;} ok=true;break;}
   uint8_t d=bestDir(posX,posY,facing,false); facing=d;
   if(realWall(posX,posY,d)){setWall(posX,posY,d,true);continue;}
   posX+=DX[d];posY+=DY[d];n++;}
  // fast run on known walls
  if(ok){ flood(true,true); if(dist[0][0]==255) ok=false; }
  total++; if(!ok)fails++; steps+=n;}
 printf("mazes %d fails %d avg cells %.1f\n",total,fails,(double)steps/total);}
