
namespace Vis
{
	enum class VisualType { PLAIN, MEAN, SLICING, ISOPHOTES, WEIGH, WEIGH2,TRANSPARENTS  };




	struct Visualization {

		Visualization() :show_control_points(true), show_solid(true), show_wireframe(false) { type = VisualType::PLAIN; }
		bool show_control_points, show_solid, show_wireframe, show_skelton;
		VisualType type;
		GLuint isophote_texture, environment_texture, current_isophote_texture, slicing_texture;

	};
}