#include "Astar.h"

namespace mawile {
	namespace AstarFunc {
		constexpr bool isDestination(int Current_z, int Current_y, int Current_x, Coordinate Dst) {
			return (Dst.z == Current_z && Dst.y == Current_y && Dst.x == Current_x);
		}

		constexpr bool isUnBlocked(std::vector<std::vector<std::vector<int>>>& Map, int Current_z, int Current_y, int Current_x) {
			return (Map[Current_z][Current_y][Current_x] == 0);
		}

		constexpr bool isInRange(int Depths, int Rows, int Cols, int Current_z, int Current_y, int Current_x) {
			return (Current_z >= 0 && Current_y >= 0
				&& Current_x >= 0 && Current_z < Depths && Current_y < Rows && Current_x < Cols);
		}

		double Distance(int X1, int Y1, int Z1, int X2, int Y2, int Z2) {
			return (double)(std::sqrt(std::pow(X1 - X2, 2) + std::pow(Y1 - Y2, 2) + std::pow(Z1 - Z2, 2)));
		}
	}

	bool Astar::Execute(int Cols, int Rows, int Depths, Coordinate Src, Coordinate Dst, std::vector<std::vector<std::vector<int>>>& Map) {
		(this->Cols) = Cols;
		(this->Rows) = Rows;
		(this->Depths) = Depths;
		(this->Map) = &Map;

		if (!AstarFunc::isInRange(Depths, Rows, Cols, Src.z, Src.y, Src.x) ||
			!AstarFunc::isInRange(Depths, Rows, Cols, Src.z, Src.y, Src.x)) return false;
		if (!AstarFunc::isUnBlocked(Map, Src.z, Src.y, Src.x) ||
			!AstarFunc::isUnBlocked(Map, Src.z, Src.y, Src.x)) return false;
		if (AstarFunc::isDestination(Src.z, Src.y, Src.x, Dst)) return true;

		std::vector<std::vector<std::vector<bool>>> closedList(Depths, std::vector<std::vector<bool>>(Rows, std::vector<bool>(Cols)));
		std::vector<std::vector<std::vector<Cell>>> CellDetails(Depths, std::vector<std::vector<Cell>>(Rows, std::vector<Cell>(Cols)));

		for (int k = 0; k < Depths; ++k) {
			for (int i = 0; i < Rows; ++i) {
				for (int j = 0; j < Cols; ++j) {
					CellDetails[k][i][j].f = CellDetails[k][i][j].g = CellDetails[k][i][j].h = AstarFunc::INF;
					CellDetails[k][i][j].parent_z = CellDetails[k][i][j].parent_y = CellDetails[k][i][j].parent_x = -1;
					closedList[k][i][j] = false;
				}
			}
		}

		int start_x = Src.x;
		int start_y = Src.y;
		int start_z = Src.z;

		CellDetails[start_z][start_y][start_x].f
			= CellDetails[start_z][start_y][start_x].g
			= CellDetails[start_z][start_y][start_x].h
			= 0.0;
		CellDetails[start_z][start_y][start_x].parent_z = start_z;
		CellDetails[start_z][start_y][start_x].parent_y = start_y;
		CellDetails[start_z][start_y][start_x].parent_x = start_x;

		std::set<CostCoordinate> openList;
		openList.insert({ 0.0, { start_z, start_y, start_x } });

		while (!openList.empty()) {
			CostCoordinate Current = *openList.begin();
			openList.erase(openList.begin());

			int Currentx = Current.coord.x;
			int Currenty = Current.coord.y;
			int Currentz = Current.coord.z;

			closedList[Currentz][Currenty][Currentx] = true;

			double Newf, Newg, Newh;

			for (std::size_t i = 0; i < 6; ++i) {
				int Nextx = Currentx + AstarFunc::Dx1[i];
				int Nexty = Currenty + AstarFunc::Dy1[i];
				int Nextz = Currentz + AstarFunc::Dz1[i];

				if (AstarFunc::isInRange(Depths, Rows, Cols, Nextz, Nexty, Nextx)) {
					if (AstarFunc::isDestination(Nextz, Nexty, Nextx, Dst)) {
						CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
						CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
						CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;
						BacktrackingMap(CellDetails, Dst);
						return true;
					}
					else if (closedList[Nextz][Nexty][Nextx] == false &&
						AstarFunc::isUnBlocked(Map, Nextz, Nexty, Nextx)) {
						Newg = CellDetails[Currentz][Currenty][Currentx].g + 1.000;
						Newh = AstarFunc::Distance(Nextx, Nexty, Nextz, Dst.x, Dst.y, Dst.z);
						Newf = Newg + Newh;

						if (CellDetails[Nextz][Nexty][Nextx].f == AstarFunc::INF ||
							CellDetails[Nextz][Nexty][Nextx].f > Newf) {
							CellDetails[Nextz][Nexty][Nextx].g = Newg;
							CellDetails[Nextz][Nexty][Nextx].h = Newh;
							CellDetails[Nextz][Nexty][Nextx].f = Newf;

							CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
							CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
							CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;

							openList.insert({ Newf, { Nextz, Nexty, Nextx } });
						}
					}
				}
			}

			for (std::size_t i = 0; i < 12; ++i) {
				int Nextx = Currentx + AstarFunc::Dx2[i];
				int Nexty = Currenty + AstarFunc::Dy2[i];
				int Nextz = Currentz + AstarFunc::Dz2[i];

				if (AstarFunc::isInRange(Depths, Rows, Cols, Nextz, Nexty, Nextx)) {
					if (AstarFunc::isDestination(Nextz, Nexty, Nextx, Dst)) {
						CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
						CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
						CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;
						BacktrackingMap(CellDetails, Dst);
						return true;
					}
					else if (closedList[Nextz][Nexty][Nextx] == false &&
						AstarFunc::isUnBlocked(Map, Nextz, Nexty, Nextx)) {
						Newg = CellDetails[Currentz][Currenty][Currentx].g + 1.414;
						Newh = AstarFunc::Distance(Nextx, Nexty, Nextz, Dst.x, Dst.y, Dst.z);
						Newf = Newg + Newh;

						if (CellDetails[Nextz][Nexty][Nextx].f == AstarFunc::INF ||
							CellDetails[Nextz][Nexty][Nextx].f > Newf) {
							CellDetails[Nextz][Nexty][Nextx].g = Newg;
							CellDetails[Nextz][Nexty][Nextx].h = Newh;
							CellDetails[Nextz][Nexty][Nextx].f = Newf;

							CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
							CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
							CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;

							openList.insert({ Newf, { Nextz, Nexty, Nextx } });
						}
					}
				}
			}

			if (this->Freeze_Z == false) {
				for (std::size_t i = 0; i < 8; ++i) {
					int Nextx = Currentx + AstarFunc::Dx3[i];
					int Nexty = Currenty + AstarFunc::Dy3[i];
					int Nextz = Currentz + AstarFunc::Dz3[i];

					if (AstarFunc::isInRange(Depths, Rows, Cols, Nextz, Nexty, Nextx)) {
						if (AstarFunc::isDestination(Nextz, Nexty, Nextx, Dst)) {
							CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
							CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
							CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;
							BacktrackingMap(CellDetails, Dst);
							return true;
						}
						else if (closedList[Nextz][Nexty][Nextx] == false &&
							AstarFunc::isUnBlocked(Map, Nextz, Nexty, Nextx)) {
							Newg = CellDetails[Currentz][Currenty][Currentx].g + 1.732;
							Newh = AstarFunc::Distance(Nextx, Nexty, Nextz, Dst.x, Dst.y, Dst.z);
							Newf = Newg + Newh;

							if (CellDetails[Nextz][Nexty][Nextx].f == AstarFunc::INF ||
								CellDetails[Nextz][Nexty][Nextx].f > Newf) {
								CellDetails[Nextz][Nexty][Nextx].g = Newg;
								CellDetails[Nextz][Nexty][Nextx].h = Newh;
								CellDetails[Nextz][Nexty][Nextx].f = Newf;

								CellDetails[Nextz][Nexty][Nextx].parent_x = Currentx;
								CellDetails[Nextz][Nexty][Nextx].parent_y = Currenty;
								CellDetails[Nextz][Nexty][Nextx].parent_z = Currentz;

								openList.insert({ Newf, { Nextz, Nexty, Nextx } });
							}
						}
					}
				}
			}
		}

		return false;
	}

	void Astar::BacktrackingMap(std::vector<std::vector<std::vector<Cell>>>& cellDetails, Coordinate Dst) {
		(this->Path).clear();
		std::stack<Coordinate> Path;
		Path.push({ Dst.z, Dst.y, Dst.x });
		(this->Path).push_back({ Dst.z, Dst.y, Dst.x });

		int Currentx = Dst.x;
		int Currenty = Dst.y;
		int Currentz = Dst.z;

		while (!(cellDetails[Currentz][Currenty][Currentx].parent_x == Currentx &&
			cellDetails[Currentz][Currenty][Currentx].parent_y == Currenty &&
			cellDetails[Currentz][Currenty][Currentx].parent_z == Currentz)) {
			int tempX = cellDetails[Currentz][Currenty][Currentx].parent_x;
			int tempY = cellDetails[Currentz][Currenty][Currentx].parent_y;
			int tempZ = cellDetails[Currentz][Currenty][Currentx].parent_z;

			Currentx = tempX;
			Currenty = tempY;
			Currentz = tempZ;

			Path.push({ Currentz, Currenty, Currentx });
			(this->Path).push_back({ Currentz, Currenty, Currentx });
		}

		while (!Path.empty()) {
			Coordinate Coord = Path.top();
			Path.pop();

			(this->Map[0][Coord.z][Coord.y][Coord.x]) = 4;
		}

		std::reverse((this->Path).begin(), (this->Path).end());
	}

	std::vector<Coordinate> Astar::GetPath() {
		return (this->Path);
	}

	Astar::Astar(bool Freeze_Z = false) noexcept {
		(this->Freeze_Z) = Freeze_Z;
	}
}