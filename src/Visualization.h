

enum class VisualType { PLAIN, MEAN, SLICING, ISOPHOTES, WEIGH, WEIGH2,TRANSPARENTS  };


struct Visualization {

	Visualization();
	bool show_control_points, show_solid, show_wireframe, show_skelton;
	VisualType type;

};