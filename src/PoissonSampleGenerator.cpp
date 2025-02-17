#include "PoissonSampleGenerator.h"

// TODO: BIG REFACTOR BROTHER UHHHH
std::vector<MyMesh::Point> PoissonSampleGenerator::poissonDisk(float radius, std::vector<SamplePoint> raw, std::vector<MyMesh::Normal>& samples_nor)
{
	std::vector<MyMesh::Point> samples;
	BBox box;
	float diff = 10;
	for (auto& r : raw)
	{
		r.pos *= diff;

	}
	for (auto& r : raw)
	{
		box.add_point(r.pos);

	}

	MyMesh::Point boxsize = box.lenghts();
	float radiussquer = radius * radius;
	MyMesh::Point gride_size = boxsize / radius;
	MyMesh::Point gride_size_int= MyMesh::Point(lrintf(gride_size[0]), lrintf(gride_size[1]), lrintf(gride_size[2]));

	gride_size_int[0] = fmaxf(gride_size_int[0], 1);
	gride_size_int[1] = fmaxf(gride_size_int[1], 1);
	gride_size_int[1] = fmaxf(gride_size_int[2], 1);

	for (auto& p : raw)
	{
		MyMesh::Point idx = box.index_grid_cell(gride_size_int, p.pos);
		Idx3 offset(gride_size_int, idx);
		p.cell_id = offset.to_linear();
	}

	std::sort(raw.begin(), raw.end(), [](const SamplePoint& lhs, const SamplePoint& rhs) {
		return lhs.cell_id < rhs.cell_id;
		});

	std::map<int, hash_data> cells;

	int last_id = -1;
	std::map<int, hash_data>::iterator last_id_it;

	for (int i = 0; i < raw.size(); ++i)
	{
		const auto& sample = raw[i];
		if (sample.cell_id == last_id)
		{
			// This sample is in the same cell as the previous, so just increase the count.  Cells are
			// always contiguous, since we've sorted raw_samples by cell ID.
			++last_id_it->second.sample_cnt;
			continue;
		}
		// This is a new cell.
		hash_data data;
		data.first_sample_idx = i;
		data.sample_cnt = 1;

		auto result = cells.insert({ sample.cell_id, data });
		last_id = sample.cell_id;
		last_id_it = result.first;
	}

	std::vector<int> neighbor_cell_offsets;
	
	Idx3 offset(gride_size_int, 0);
	for (int x = -1; x <= +1; ++x)
	{
		for (int y = -1; y <= +1; ++y)
		{
			for (int z = -1; z <= +1; ++z)
			{
				offset.set_3d(MyMesh::Point(x, y, z));
				neighbor_cell_offsets.push_back(offset.to_linear());
			}
		}
	}
	int max_trials = 5;
	for (int trial = 0; trial < max_trials; ++trial)
	{
		// Create sample points for each entry in cells.
		for (auto& it : cells)
		{
			int cell_id = it.first;
			hash_data& data = it.second;

			// This cell's raw sample points start at first_sample_idx.  On trial 0, try the first one.
			// On trial 1, try first_sample_idx + 1.
			int next_sample_idx = data.first_sample_idx + trial;
			if (trial >= data.sample_cnt)
			{
				// There are no more points to try for this cell.
				continue;
			}
			const auto& candidate = raw[next_sample_idx];

			// See if this point conflicts with any other points in this cell, or with any points in
			// neighboring cells.  Note that it's possible to have more than one point in the same cell.
			bool conflict = false;
			for (int neighbor_offset : neighbor_cell_offsets)
			{
				int neighbor_cell_id = cell_id + neighbor_offset;
				const auto& it = cells.find(neighbor_cell_id);
				if (it == cells.end())
					continue;

				const hash_data& neighbor = it->second;
				for (const auto& sample : neighbor.poisson_samples)
				{
					float distance = approximate_geodesic_distance(sample.pos, candidate.pos, sample.normal, candidate.normal);
					if (distance < radiussquer)
					{
						// The candidate is too close to this existing sample.
						conflict = true;
						break;
					}
				}
				if (conflict)
					break;
			}

			if (conflict)
				continue;
			// Store the new sample.
			data.poisson_samples.emplace_back();
			poisson_sample& new_sample = data.poisson_samples.back();
			new_sample.pos = candidate.pos;
			new_sample.normal = candidate.normal;
		}
	}
	for (const auto it : cells)
	{
		for (const auto& sample : it.second.poisson_samples)
		{
			samples.push_back(sample.pos/ diff);
			samples_nor.push_back(sample.normal);
		}
	}

	return samples;
}





// TODO: BIG REFACTOR BROTHER UHHHH
float PoissonSampleGenerator::generateSamples(int num_samples, MyMesh mesh_, std::vector<SamplePoint>& samples)
{

	std::vector<double> tri_area;
	std::map<double, MyMesh::FaceHandle> area_sum_to_index;


	for (auto f : mesh_.faces())
	{
		double area = mesh_.calc_sector_area(mesh_.halfedge_handle(f));
		tri_area.push_back(area);

	}

	float max_area_sum = 0;
	int j = 0;
	for (auto f : mesh_.faces())
	{
		area_sum_to_index[max_area_sum] = f;
		max_area_sum += tri_area[j];
		j++;
	}
	for (int i = 0; i < num_samples; i++)
	{
		float r = random_float(max_area_sum);
		auto it = area_sum_to_index.upper_bound(r);
		assert(it != area_sum_to_index.begin());
		--it;
		auto tri = it->second;
		std::vector<MyMesh::Point> poi;
		std::vector<MyMesh::Normal> nor;
		float u = random_float(1);
		float v = random_float(1);
		for (auto v : mesh_.fv_range(tri))
		{
			poi.push_back(mesh_.point(v));
			nor.push_back(mesh_.normal(v));

		}
		if (poi.size() == 3 && nor.size() == 3)
		{
			MyMesh::Point pos = poi[0] * (1 - sqrt(u)) +
				poi[1] * (sqrt(u) * (1 - v)) +
				poi[2] * (v * sqrt(u));


			MyMesh::Normal normal = nor[0] * (1 - sqrt(u)) +
				nor[1] * (sqrt(u) * (1 - v)) +
				nor[2] * (v * sqrt(u));

			SamplePoint sp = SamplePoint(pos, normal);

			sp.cell_id = -1;
			sp.tri = tri;

			samples.push_back(sp);
		}


	}


	return max_area_sum;

}


float PoissonSampleGenerator::approximate_geodesic_distance(MyMesh::Point p1, MyMesh::Point p2, MyMesh::Normal n1, MyMesh::Normal  n2)
{
	n1.normalize();
	n2.normalize();

	MyMesh::Point v = (p2 - p1);
	v.normalize();

	float c1 = n1 | v;
	float c2 = n2 | v;
	float result = (p2 - p1) | (p2 - p1);
	// Check for division by zero:
	if (fabs(c1 - c2) > 0.0001)
		result *= (asin(c1) - asin(c2)) / (c1 - c2);

	return result;
}