//矩阵位置坐标
struct Rect {
    Rect(int x, int y, int width, int height)
        : x(x), y(y), width(width), height(height)
    {}
    int x;          //矩形左上角x坐标
    int y;          //矩形左上角y坐标
    int width;      //矩形宽度
    int height;     //矩形高度
};

// 判断两个矩形是否叠加
bool isOverlap(const Rect &rc1, const Rect &rc2)
{
    if (rc1.x + rc1.width  > rc2.x &&
        rc2.x + rc2.width  > rc1.x &&
        rc1.y + rc1.height > rc2.y &&
        rc2.y + rc2.height > rc1.y
       )
        return true;
    else
        return false;
}

// 获取叠加矩形数量
// @param rectSet存储10000个矩形的位置信息
int getOverlapRectNums(vector<Rect> &rectSet)
{
	int rectNums = rectSet.size();
	vector<bool> = overlapSign(rectSet.size(), false);
	int overLapNums = 0;
	for(int i=0; i<rectNums; i++)
	{
		if (overlapSign[i])
			continue;
		for(int j=0; j<rectNums; j++)
		{
			if (i==j)
				continue;
			bool isOverlap = isOverlap(rectSet[i], rectSet[j]);
			if (isOverlap)
			{
				overlapSign[i] = true;
				overlapSign[j] = true;
				overLapNums++;
				break;
			}
		}
	}
	return overLapNums==0?0:overLapNums+1;
}