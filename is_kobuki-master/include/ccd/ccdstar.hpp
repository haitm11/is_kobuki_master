#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

int ROWS;
int COLS;
// TODO cac ban do khac nhau co RM khac nhau
#define RM 1  // ban kinh robot = 2 cell

typedef struct Cells { 
  float g;  
  float gv2;
  int row;
  int col;
  int visited;
  int visitedReal;
  int bpRow,bpCol,bpRowv2,bpColv2;  // toa do cua back point
  int isObExtend;           // co phai vat can mo rong khong
  int isObReal;           // co phai vat can that su khong
  int visitedDStar;
  int belongCcdPath;          
}cell; 

/* Xay dung QUEUE */
typedef struct Node {
    cell Data;
    struct Node *Next;
}node;

typedef struct Queue {
    node *Front, *Rear;
    int count;
}queue;

void Init(queue* Q) {
    Q->Front = Q->Rear = NULL;
    Q->count = 0;
}

int Isempty(queue Q) {
    if (Q.count == 0) 
        return 1;
    return 0;
}

node *MakeNode(cell x) {
    node *P = (node*) malloc(sizeof(node));
    P->Next = NULL;
    P->Data = x;
    return P;
}

void PushAndSort(queue *Q, cell x) {
    node *temp, *temp2;
    node *p = MakeNode(x); 
    if (Isempty(*Q)) {
        Q->Front = Q->Rear = p;
    } else { 
      temp = Q->Front;
      while(1) {
        if((temp->Data).g > (p->Data).g) {
          if(temp == Q->Front) {
            p->Next = Q->Front;
            Q->Front = p;
          } else {
            p->Next = temp;
            temp2->Next = p;
          }
          break;
        } else {
          if(temp == Q->Rear) {
            Q->Rear->Next = p;
              Q->Rear = p;
            break;
          }
          temp2 = temp;
          temp = temp->Next;
        }
      }
    }
    Q->count++ ; 
}

void PushAndSortv2(queue *Q, cell x) {
    node *temp, *temp2;
    node *p = MakeNode(x); 
    if (Isempty(*Q)) {
        Q->Front = Q->Rear = p;
    } else { 
      temp = Q->Front;
      while(1) {
        if((temp->Data).gv2 > (p->Data).gv2) {
          if(temp == Q->Front) {
            p->Next = Q->Front;
            Q->Front = p;
          } else {
            p->Next = temp;
            temp2->Next = p;
          }
          break;
        } else {
          if(temp == Q->Rear) {
            Q->Rear->Next = p;
              Q->Rear = p;
            break;
          }
          temp2 = temp;
          temp = temp->Next;
        }
      }
    }
    Q->count++ ; 
}

void Push(queue *Q, cell x) {
    node *p = MakeNode(x); 
    if (Isempty(*Q)) {
        Q->Front = Q->Rear = p;
    } else { 
      Q->Rear->Next = p;
      Q->Rear = p;
    }
    Q->count++ ; 
}

cell Pop(queue *Q) {
    cell x = Q->Front->Data;
    if (Q->count == 1)
        Init(Q);
    else {
        Q->Front = Q->Front->Next;
      Q->count--;
    }
    return x; 
}

int checkContainQueue(queue *Q, cell c) {
  node *temp;
  temp = Q->Front;
  if(Isempty(*Q)) return 0;
  while(1) {
    if((temp->Data).row == c.row && (temp->Data).col == c.col) return 1;
    if(temp == Q->Rear) break;
    temp = temp->Next;
  }
  return 0;
}
/*-----------------*/
void setRowsAndCols(int r, int c);

int checkStopDStar(cell **cells, cell nextCell, queue *ccdPathQueue);

int dstar(cell **cells, int firstRow, int firstCol);

cell dstarV2(cell **cells, queue *ccdPathQueue,int curentRow, int curentCol);

void buildObExtend(cell **cells);

cell findNextCellByPosition(cell **cells, cell currentCell, int position);

cell findNextCell(cell **cells, cell currentCell);

void setMaskVisited(cell **cells, int row, int col);

queue coverage(cell **cells, int firstRow, int firstCol);

int checkContainOb(cell **cells, int row, int col);

node *findCcdPathQueue(queue *ccdPathQueue, int row, int col);

/* lay vi tri cua cell 2 so voi cell 1*/
int getCellPosition(int cell1Row, int cell1Col, int cell2Row, int cell2Col);

int checkSpecialPoint(node *temp, node* temp2, Queue ccdPathQueue);

node *findLastCommonPoint(queue *p1, queue *p2);

