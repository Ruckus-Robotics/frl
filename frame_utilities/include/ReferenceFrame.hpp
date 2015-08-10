

class ReferenceFrame
{
public:
	ReferenceFrame(string frameName, ReferenceFrame parentFrame);
private:
	ReferenceFrame *parentFrame;
}