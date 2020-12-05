#include "../../include/ccd/ccdstar.hpp"

void setRowsAndCols(int r, int c) {
	ROWS = r;
	COLS = c;
}
int checkStopDStar(cell **cells, cell nextCell, queue *ccdPathQueue) {
	int row, col, m, n;
	row = nextCell.row;
	col = nextCell.col;

	if(checkContainQueue(ccdPathQueue, nextCell)) return 0;
	if(cells[row-1][col].isObExtend == 0 && cells[row+1][col].isObExtend == 0 &&
		cells[row][col-1].isObExtend == 0 && cells[row][col+1].isObExtend == 0) return 0;
	for(m = row-RM; m <= row+RM; m++) 
		for(n = col-RM; n <= col+RM; n++)
			if(cells[m][n].visited == 0) return 1;
	return 0;
}

int dstar(cell **cells, int firstRow, int firstCol) {
	int row, col, i, j;
	row = firstRow;
	col = firstCol;
	queue que;
	Init(&que);

	for(i = 0; i < ROWS; i++)
		for(j = 0; j < COLS; j++) {
			cells[i][j].visitedDStar = 0;
		}

	cells[row][col].g = 0;
	cells[row][col].visitedDStar = 1;
	cells[row][col].bpRow = firstRow;
	cells[row][col].bpCol = firstCol;
	PushAndSort(&que, cells[row][col]);

	while(Isempty(que) == 0) {
		cell nextCell = Pop(&que);
		row = nextCell.row;
		col = nextCell.col;
		if(cells[row-1][col-1].isObExtend==0 && cells[row-1][col-1].visitedDStar==0) {
			cells[row-1][col-1].g = cells[row][col].g + 1.4;
			cells[row-1][col-1].visitedDStar = 1;
			cells[row-1][col-1].bpRow = row;
			cells[row-1][col-1].bpCol = col;
			PushAndSort(&que, cells[row-1][col-1]);
		}
		if(cells[row-1][col].isObExtend==0 && cells[row-1][col].visitedDStar==0) {
			cells[row-1][col].g = cells[row][col].g + 1.0;
			cells[row-1][col].visitedDStar = 1;
			cells[row-1][col].bpRow = row;
			cells[row-1][col].bpCol = col;
			PushAndSort(&que, cells[row-1][col]);
		}
		if(cells[row-1][col+1].isObExtend==0 && cells[row-1][col+1].visitedDStar==0) {
			cells[row-1][col+1].g = cells[row][col].g + 1.4;
			cells[row-1][col+1].visitedDStar = 1;
			cells[row-1][col+1].bpRow = row;
			cells[row-1][col+1].bpCol = col;
			PushAndSort(&que, cells[row-1][col+1]);
		}
		if(cells[row][col-1].isObExtend==0 && cells[row][col-1].visitedDStar==0) {
			cells[row][col-1].g = cells[row][col].g + 1.0;
			cells[row][col-1].visitedDStar = 1;
			cells[row][col-1].bpRow = row;
			cells[row][col-1].bpCol = col;
			PushAndSort(&que, cells[row][col-1]);
		}
		if(cells[row][col+1].isObExtend==0 && cells[row][col+1].visitedDStar==0) {
			cells[row][col+1].g = cells[row][col].g + 1.0;
			cells[row][col+1].visitedDStar = 1;
			cells[row][col+1].bpRow = row;
			cells[row][col+1].bpCol = col;
			PushAndSort(&que, cells[row][col+1]);
		}
		if(cells[row+1][col-1].isObExtend==0 && cells[row+1][col-1].visitedDStar==0) {
			cells[row+1][col-1].g = cells[row][col].g + 1.4;
			cells[row+1][col-1].visitedDStar = 1;
			cells[row+1][col-1].bpRow = row;
			cells[row+1][col-1].bpCol = col;
			PushAndSort(&que, cells[row+1][col-1]);
		}
		if(cells[row+1][col].isObExtend==0 && cells[row+1][col].visitedDStar==0) {
			cells[row+1][col].g = cells[row][col].g + 1.0;
			cells[row+1][col].visitedDStar = 1;
			cells[row+1][col].bpRow = row;
			cells[row+1][col].bpCol = col;
			PushAndSort(&que, cells[row+1][col]);
		}
		if(cells[row+1][col+1].isObExtend==0 && cells[row+1][col+1].visitedDStar==0) {
			cells[row+1][col+1].g = cells[row][col].g + 1.4;
			cells[row+1][col+1].visitedDStar = 1;
			cells[row+1][col+1].bpRow = row;
			cells[row+1][col+1].bpCol = col;
			PushAndSort(&que, cells[row+1][col+1]);
		}
	} 
	return 1;
}

cell dstarV2(cell **cells, queue *ccdPathQueue,int curentRow, int curentCol) {
	int row, col, i, j;
	row = curentRow;
	col = curentCol;
	queue que;
	Init(&que);

	for(i = 0; i < ROWS; i++)
		for(j = 0; j < COLS; j++) {
			cells[i][j].visitedDStar = 0;
		}

	cells[row][col].gv2 = 0;
	cells[row][col].visitedDStar = 1;
	cells[row][col].bpRowv2 = row;
	cells[row][col].bpColv2 = col;

	PushAndSortv2(&que, cells[row][col]);

	while(Isempty(que) == 0) {
		cell nextCell = Pop(&que);
		if(nextCell.row != curentRow || nextCell.col != curentCol) {
			if(nextCell.visited == 0) return nextCell;
			if(checkStopDStar(cells, nextCell, ccdPathQueue)) return nextCell;
		}
		row = nextCell.row;
		col = nextCell.col;
		if(cells[row-1][col-1].isObExtend == 0 && cells[row-1][col-1].visitedDStar == 0) {
			cells[row-1][col-1].gv2 = cells[row][col].gv2 + 1.4;
			cells[row-1][col-1].visitedDStar = 1;
			cells[row-1][col-1].bpRowv2 = row;
			cells[row-1][col-1].bpColv2 = col;
			PushAndSortv2(&que, cells[row-1][col-1]);
		}
		if(cells[row-1][col].isObExtend==0 && cells[row-1][col].visitedDStar==0) {
			cells[row-1][col].gv2 = cells[row][col].gv2 + 1.0;
			cells[row-1][col].visitedDStar = 1;
			cells[row-1][col].bpRowv2 = row;
			cells[row-1][col].bpColv2 = col;
			PushAndSortv2(&que, cells[row-1][col]);
		}
		if(cells[row-1][col+1].isObExtend==0 && cells[row-1][col+1].visitedDStar==0) {
			cells[row-1][col+1].gv2 = cells[row][col].gv2 + 1.4;
			cells[row-1][col+1].visitedDStar = 1;
			cells[row-1][col+1].bpRowv2 = row;
			cells[row-1][col+1].bpColv2 = col;
			PushAndSortv2(&que, cells[row-1][col+1]);
		}
		if(cells[row][col-1].isObExtend==0 && cells[row][col-1].visitedDStar==0) {
			cells[row][col-1].gv2 = cells[row][col].gv2 + 1.0;
			cells[row][col-1].visitedDStar = 1;
			cells[row][col-1].bpRowv2 = row;
			cells[row][col-1].bpColv2 = col;
			PushAndSortv2(&que, cells[row][col-1]);
		}
		if(cells[row][col+1].isObExtend==0 && cells[row][col+1].visitedDStar==0) {
			cells[row][col+1].gv2 = cells[row][col].gv2 + 1.0;
			cells[row][col+1].visitedDStar = 1;
			cells[row][col+1].bpRowv2 = row;
			cells[row][col+1].bpColv2 = col;
			PushAndSortv2(&que, cells[row][col+1]);
		}
		if(cells[row+1][col-1].isObExtend==0 && cells[row+1][col-1].visitedDStar==0) {
			cells[row+1][col-1].gv2 = cells[row][col].gv2 + 1.4;
			cells[row+1][col-1].visitedDStar = 1;
			cells[row+1][col-1].bpRowv2 = row;
			cells[row+1][col-1].bpColv2 = col;
			PushAndSortv2(&que, cells[row+1][col-1]);
		}
		if(cells[row+1][col].isObExtend==0 && cells[row+1][col].visitedDStar==0) {
			cells[row+1][col].gv2 = cells[row][col].gv2 + 1.0;
			cells[row+1][col].visitedDStar = 1;
			cells[row+1][col].bpRowv2 = row;
			cells[row+1][col].bpColv2 = col;
			PushAndSortv2(&que, cells[row+1][col]);
		}
		if(cells[row+1][col+1].isObExtend==0 && cells[row+1][col+1].visitedDStar==0) {
			cells[row+1][col+1].gv2 = cells[row][col].gv2 + 1.4;
			cells[row+1][col+1].visitedDStar = 1;
			cells[row+1][col+1].bpRowv2 = row;
			cells[row+1][col+1].bpColv2 = col;
			PushAndSortv2(&que, cells[row+1][col+1]);
		}
	} 
	cell c;
	c.g = -1;
	return c;
}

void buildObExtend(cell **cells) {
	int i,j,m,n;
	for(i=0;i<ROWS;i++) {
		for(j=0;j<COLS;j++) {
			if(cells[i][j].isObReal!=0) {
				for(m=i-RM;m<=i+RM;m++) {
					for(n=j-RM;n<=j+RM;n++){
						if(m>=0 && m<ROWS && n>=0 && n<COLS)
							cells[m][n].isObExtend = 1;
					}
				}
			}
		}
	}
}

cell findNextCellByPosition(cell **cells, cell currentCell, int position) {
	cell c;
	c.g = -1;
	int row, col;
	if (position == 0) {	
		row = currentCell.row-1-2*RM;
		col = currentCell.col;
	}
	if (position == 1) {			// LEFT
		row = currentCell.row;
		col = currentCell.col-1-2*RM;
	}
	if (position == 2) {			// DOWN
		row = currentCell.row+1+2*RM;
		col = currentCell.col;
	}
	if (position == 3) {			// RIGHT
		row = currentCell.row;
		col = currentCell.col+1+2*RM;
	}
	int m, n;
	for(m = row-RM; m <= row+RM; m++) {
		for(n = col-RM; n <= col+RM; n++){
			if(m >= 0 && m < ROWS && n >= 0 && n < COLS) {
				if(cells[m][n].isObReal != 0) return c;
				if(cells[m][n].visited == 1) return c;
			}
			else return c;
		}
	}
	return cells[row][col];
}

cell findNextCell(cell **cells, cell currentCell) {
	cell cup, cleft, cdown, cright, c, cmin;
	c.g = -1;
	cup = findNextCellByPosition(cells, currentCell, 0);
	cleft = findNextCellByPosition(cells, currentCell, 1);
	cdown = findNextCellByPosition(cells, currentCell, 2);
	cright = findNextCellByPosition(cells, currentCell, 3);
	if(cup.g == -1 && cleft.g == -1 && cdown.g == -1 && cright.g == -1) return c;
	else {
		if(cup.g != -1) cmin = cup;
		if(cleft.g != -1) cmin = cleft;
		if(cdown.g != -1) cmin = cdown;
		if(cright.g != -1) cmin = cright;

		if(cup.g != -1 && cup.g < cmin.g) cmin = cup;
		if(cleft.g != -1 && cleft.g < cmin.g) cmin = cleft;
		if(cdown.g != -1 && cdown.g < cmin.g) cmin = cdown;
		if(cright.g != -1 && cright.g < cmin.g) cmin = cright;

		return cmin;
	}
}

void setMaskVisited(cell **cells, int row, int col) {
	int m, n;
	for(m = row-RM; m <= row+RM; m++) 
		for(n = col-RM; n <= col+RM; n++) 
			cells[m][n].visited = 1;
}

queue coverage(cell **cells, int firstRow, int firstCol) {
  int curentRow = firstRow;
	int curentCol = firstCol;
	int temp, k;
	k = 0;
	cell nextCell, c;
	int arr[10000][2];
	queue ccdPathQueue;
	Init(&ccdPathQueue);

	setMaskVisited(cells,curentRow,curentCol);
	while (1) {
		// Sleep(350);
		nextCell = findNextCell(cells, cells[curentRow][curentCol]);
		if(nextCell.g != -1) {
			setMaskVisited(cells,nextCell.row,nextCell.col);
			if(curentRow == nextCell.row) {
				temp = curentCol;
				while(1) {
					Push(&ccdPathQueue, cells[curentRow][temp]);
					cells[curentRow][temp].belongCcdPath = 1;
					if(temp == nextCell.col) break;
					if(curentCol > nextCell.col) temp--; else temp++;
				}
			}
			if(curentCol == nextCell.col) {
				temp = curentRow;
				while(1) {
					Push(&ccdPathQueue, cells[temp][curentCol]);
					cells[temp][curentCol].belongCcdPath = 1;
					if(temp == nextCell.row) break;
					if(curentRow > nextCell.row) temp--; else temp++;
				}
			}
			curentRow = nextCell.row;
			curentCol = nextCell.col;
		} else {
			nextCell = dstarV2(cells, &ccdPathQueue, curentRow, curentCol);
			if(nextCell.g == -1) break;
			else {
				setMaskVisited(cells,nextCell.row,nextCell.col);

				c = nextCell;
				while(1) {
					if(c.row == curentRow && c.col == curentCol) break;
					arr[k][0] = c.row;
					arr[k][1] = c.col;
					k++;
					c = cells[c.bpRowv2][c.bpColv2];
				}

				while(1) {
					Push(&ccdPathQueue, cells[arr[k-1][0]][arr[k-1][1]]);
					cells[arr[k-1][0]][arr[k-1][1]].belongCcdPath = 1;
					k--;
					if(k==0) break;
				}
				curentRow = nextCell.row;
				curentCol = nextCell.col;
			}
		}
	}

  node *temp1, *temp2;
  temp1 = ccdPathQueue.Front;

  while(1) {
	temp2 = temp1;
	temp1 = temp1->Next;
	if(temp1 == ccdPathQueue.Rear) break;
	if(((temp2->Data).row == (temp1->Data).row) && ((temp2->Data).col == (temp1->Data).col)) {
	  temp2->Next = temp1->Next;
	}
  }
  
  return ccdPathQueue;
}

int checkContainOb(cell **cells, int row, int col) {
	int i,j;
	for(i=row-RM;i<=row+RM;i++) 
		for(j=col-RM;j<=col+RM;j++)
			if(cells[i][j].isObReal != 0) return 1;
	return 0;
}

node *findCcdPathQueue(queue *ccdPathQueue, int row, int col) {
	node *temp;
	temp = ccdPathQueue->Front;
	while(1) {
		if((temp->Data).row == row && (temp->Data).col == col) return temp;
		if(temp == ccdPathQueue->Rear) return NULL;
		temp = temp->Next;
	}
}

/* lay vi tri cua cell 2 so voi cell 1*/
int getCellPosition(int cell1Row, int cell1Col, int cell2Row, int cell2Col){
  if(cell1Row == cell2Row && cell1Col > cell2Col) return 1;
  if(cell1Row == cell2Row && cell1Col < cell2Col) return 3;
  if(cell1Row > cell2Row && cell1Col == cell2Col) return 0;
  if(cell1Row < cell2Row && cell1Col == cell2Col) return 2;
  if(cell1Row > cell2Row && cell1Col > cell2Col) return 5;
  if(cell1Row < cell2Row && cell1Col > cell2Col) return 6;
  if(cell1Row > cell2Row && cell1Col < cell2Col) return 4;
  if(cell1Row < cell2Row && cell1Col < cell2Col) return 7;
  return -1;
}

// ham nay ap dung rieng cho bai toan CCD ung dung trong robot hut bui, Neu chi la CCD thuan thi khong can ham nay
int checkSpecialPoint(node *temp, node* temp2, Queue ccdPathQueue) {
  if(temp == ccdPathQueue.Rear) return 1;
  if(temp == ccdPathQueue.Front) return 0;
  float r,r2,rn,c,c2,cn,d1,d2;
  r = (float)(temp->Data).row;
  r2 = (float)(temp2->Data).row;
  c = (float)(temp->Data).col;
  c2 = (float)(temp2->Data).col;
  rn = (float)((temp->Next)->Data).row;
  cn = (float)((temp->Next)->Data).col;
  d1 = (r2-r)/(c2-c);
  d2 = (r-rn)/(c-cn);

  if (r2 == rn && c2 == cn) return 1;  // diem quay dau
  if (d1 == d2) return 0; // thang hang
  return 1;
}

node *findLastCommonPoint(queue *p_old, queue *p_new) {
	node *n1, *n2;
	int r1,c1,r2,c2;
	n1 = p_old->Front;
	n2 = p_new->Front;

	while((n1 != p_old->Rear) && (n2 != p_new->Rear)) {
		r1 = ((n1->Next)->Data).row;
		c1 = ((n1->Next)->Data).col;
		r2 = ((n2->Next)->Data).row;
		c2 = ((n2->Next)->Data).col;
		if(r1 != r2 || c1 != c2) return n2;
		n1 = n1->Next;
		n2 = n2->Next;
	}
	return NULL;
}