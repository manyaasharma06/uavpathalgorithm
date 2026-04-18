#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>
#include <math.h>
#include <time.h>

#define ROW 5
#define COL 5
#define MAX_NODES 10000

typedef struct {
    int x, y;
} Point;

typedef struct {
    double f, g;
} Cell;

typedef struct {
    Point pos;
    int parent;
} Node;

int grid[ROW][COL];

int isValid(int x, int y) {
    return x >= 0 && x < ROW && y >= 0 && y < COL;
}

int isFree(int x, int y) {
    return grid[x][y] == 0;
}

void inputGrid() {
    printf("Enter grid (0 = free, 1 = obstacle):\n");
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            scanf("%d", &grid[i][j]);
}

// DIJKSTRA 
void dijkstra(Point start, Point end) {
    int dist[ROW][COL], visited[ROW][COL] = {0};
    int nodes = 0;

    for (int i=0;i<ROW;i++)
        for (int j=0;j<COL;j++)
            dist[i][j]=INT_MAX;

    dist[start.x][start.y]=0;

    clock_t t0 = clock();

    while (1) {
        int min=INT_MAX,x=-1,y=-1;

        for (int i=0;i<ROW;i++)
            for (int j=0;j<COL;j++)
                if(!visited[i][j] && dist[i][j]<min){
                    min=dist[i][j]; x=i; y=j;
                }

        if(x==-1) break;

        visited[x][y]=1;
        nodes++;

        if(x==end.x && y==end.y) break;

        int dx[]={-1,1,0,0};
        int dy[]={0,0,-1,1};

        for(int i=0;i<4;i++){
            int nx=x+dx[i], ny=y+dy[i];
            if(isValid(nx,ny)&&isFree(nx,ny)&&!visited[nx][ny]){
                if(dist[x][y]+1<dist[nx][ny])
                    dist[nx][ny]=dist[x][y]+1;
            }
        }
    }

    clock_t t1 = clock();

    printf("\n--- Dijkstra ---\n");
    if(dist[end.x][end.y]==INT_MAX)
        printf("No Path Found\n");
    else
        printf("Shortest Distance: %d\n", dist[end.x][end.y]);

    printf("Nodes Explored: %d\n", nodes);
    printf("Time Taken: %f sec\n", (double)(t1 - t0)/CLOCKS_PER_SEC);
}

//A*
double heuristic(int x,int y,Point end){
    return sqrt((x-end.x)*(x-end.x)+(y-end.y)*(y-end.y));
}

void aStar(Point start, Point end){
    Cell cell[ROW][COL];
    int closed[ROW][COL]={0}, nodes=0;

    for(int i=0;i<ROW;i++)
        for(int j=0;j<COL;j++){
            cell[i][j].f=FLT_MAX;
            cell[i][j].g=FLT_MAX;
        }

    cell[start.x][start.y].f=0;
    cell[start.x][start.y].g=0;

    clock_t t0 = clock();

    while(1){
        double minF=FLT_MAX;
        int x=-1,y=-1;

        for(int i=0;i<ROW;i++)
            for(int j=0;j<COL;j++)
                if(!closed[i][j] && cell[i][j].f<minF){
                    minF=cell[i][j].f;
                    x=i;y=j;
                }

        if(x==-1) break;

        closed[x][y]=1;
        nodes++;

        if(x==end.x && y==end.y){
            clock_t t1 = clock();

            printf("\n--- A* ---\nPath Found\n");
            printf("Nodes Explored: %d\n",nodes);
            printf("Time Taken: %f sec\n", (double)(t1 - t0)/CLOCKS_PER_SEC);
            return;
        }

        int dx[]={-1,1,0,0};
        int dy[]={0,0,-1,1};

        for(int i=0;i<4;i++){
            int nx=x+dx[i], ny=y+dy[i];

            if(isValid(nx,ny)&&isFree(nx,ny)&&!closed[nx][ny]){
                double gNew=cell[x][y].g+1;
                double fNew=gNew+heuristic(nx,ny,end);

                if(cell[nx][ny].f>fNew){
                    cell[nx][ny].f=fNew;
                    cell[nx][ny].g=gNew;
                }
            }
        }
    }

    clock_t t1 = clock();
    printf("\n--- A* ---\nNo Path Found\n");
    printf("Nodes Explored: %d\n",nodes);
    printf("Time Taken: %f sec\n", (double)(t1 - t0)/CLOCKS_PER_SEC);
}

Node tree[MAX_NODES];
int node_count=0;

double dist(Point a,Point b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

int nearest(Point p){
    int best=0;
    double d=dist(tree[0].pos,p);

    for(int i=1;i<node_count;i++){
        double nd=dist(tree[i].pos,p);
        if(nd<d){ d=nd; best=i;}
    }
    return best;
}

Point steer(Point from,Point to){
    Point p=from;
    if(to.x>from.x)p.x++;
    else if(to.x<from.x)p.x--;
    else if(to.y>from.y)p.y++;
    else if(to.y<from.y)p.y--;
    return p;
}

void rrt(Point start,Point goal){
    node_count=0;
    tree[node_count++] = (Node){start,-1};

    int nodes=0, found=0;

    clock_t t0 = clock();

    for(int i=0;i<MAX_NODES;i++){
        Point randp={rand()%ROW,rand()%COL};

        if(!isFree(randp.x,randp.y)) continue;

        int near=nearest(randp);
        Point newp=steer(tree[near].pos,randp);

        if(!isFree(newp.x,newp.y)) continue;

        tree[node_count++] = (Node){newp,near};
        nodes++;

        if(newp.x==goal.x && newp.y==goal.y){
            found=1;
            break;
        }
    }

    clock_t t1 = clock();

    printf("\n--- RRT ---\n");
    if(found)
        printf("Path Found\n");
    else
        printf("No Path Found\n");

    printf("Nodes Explored: %d\n",nodes);
    printf("Time Taken: %f sec\n", (double)(t1 - t0)/CLOCKS_PER_SEC);
}

// D* 
void dStar(Point start, Point goal){
    int cost[ROW][COL], visited[ROW][COL]={0};
    int nodes=0;

    for(int i=0;i<ROW;i++)
        for(int j=0;j<COL;j++)
            cost[i][j]=INT_MAX;

    cost[goal.x][goal.y]=0;

    clock_t t0 = clock();

    while(1){
        int min=INT_MAX,x=-1,y=-1;

        for(int i=0;i<ROW;i++)
            for(int j=0;j<COL;j++)
                if(!visited[i][j] && cost[i][j]<min){
                    min=cost[i][j];
                    x=i;y=j;
                }

        if(x==-1) break;

        visited[x][y]=1;
        nodes++;

        int dx[]={-1,1,0,0};
        int dy[]={0,0,-1,1};

        for(int i=0;i<4;i++){
            int nx=x+dx[i], ny=y+dy[i];

            if(isValid(nx,ny)&&isFree(nx,ny)){
                if(cost[x][y]+1 < cost[nx][ny]){
                    cost[nx][ny]=cost[x][y]+1;
                }
            }
        }
    }

    clock_t t1 = clock();

    printf("\n--- D* ---\n");
    if(cost[start.x][start.y]==INT_MAX)
        printf("No Path Found\n");
    else
        printf("Cost from start: %d\n", cost[start.x][start.y]);

    printf("Nodes Explored: %d\n",nodes);
    printf("Time Taken: %f sec\n", (double)(t1 - t0)/CLOCKS_PER_SEC);
}

//MAIN 
int main(){
    srand(time(NULL));

    Point start, goal;

    inputGrid();

    printf("Enter start (x y): ");
    scanf("%d %d", &start.x, &start.y);

    printf("Enter goal (x y): ");
    scanf("%d %d", &goal.x, &goal.y);

    if (!isFree(start.x, start.y) || !isFree(goal.x, goal.y)) {
        printf("Invalid start/goal (blocked)\n");
        return 0;
    }

    dijkstra(start,goal);
    aStar(start,goal);
    rrt(start,goal);
    dStar(start,goal);

    return 0;
}