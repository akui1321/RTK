#include "data.h"
#include "SPP.h"

SPP::SPP()
{
	Pos_0[0] = Pos_0[1] = Pos_0[2] = 0.0;
	dt[0] = dt[1] = 0.0;
	ValidSat = 0;
	GPScount = 0;
	BDScount = 0;
	Iterator = 0;
	paraNum = 5;
	PDOP = 0.0;
	sigma_0 = 0.0;
}

//Compute Receiver Position
bool SPP::SolvePosition(EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph)
{
	if ((GPSEph.size() == 0) && (BDSEph.size() == 0))
	{
		std::cout << "Insufficient satellite ephemeris, unable to compute!" << std::endl;
		return false;
	}

	double mag = 0;
	Pos_0[0] = Pos_0[1] = Pos_0[2] = 0.0;
	dt[0] = dt[1] = 0.0;
	Iterator = 0;

	//Iterative Calculation
	do {
		Iterator++;

		//Compute Satellite Position
		SatellitePositionAndClockError SPCE;
		range.SatMidRes = SPCE.CalculatePVCE(range, GPSEph, BDSEph, 1);
		range.SatMidRes = SPCE.CalculatePVCE(range, GPSEph, BDSEph, 2);

		ValidSat = 0;
		GPScount = 0;
		BDScount = 0;

		if (range.SatMidRes.size() == 0)
		{
			std::cout << "Insufficient observations, unable to compute!" << std::endl;
			return false;
		}

		//Compute Valid Satellite Count
		for (int i = 0; i < range.SatMidRes.size(); ++i)
		{
			if ((!range.satRange[i].valid) || (!range.SatMidRes[i].Valid))
			{
				continue;
			}
			else
			{
				ValidSat++;
			}
		}

		if (ValidSat < 4)
		{
			std::cout << "Insufficient observations, unable to compute!" << std::endl;
			return false;
		}

		Matrix B(ValidSat, 5), l(ValidSat, 1), P(ValidSat, ValidSat);
		int index = 0;
		for (int i = 0; i < ValidSat; ++i)
		{
			P(i, i) = 1;
		}
		
		for (int i = 0; i < range.SatMidRes.size(); ++i)
		{
			if ((!range.satRange[i].valid) || (!range.SatMidRes[i].Valid))
			{
				continue;
			}
			else
			{
				index++;
			}

			//Earth Rotation Correction
			Matrix Pos(3, 1), RotaResultPos(3, 1), RotaResultVel(3, 1);
			RotationCorrection(Pos, range.SatMidRes[i], range.satRange[i].system);

			for (int j = 0; j < 3; ++j)
			{
				RotaResultPos(j, 0) = range.SatMidRes[i].SatPos[j];
				RotaResultVel(j, 0) = range.SatMidRes[i].SatVel[j];
			}

			//Compute Satellite Azimuth and Elevation
			Coordinate BLH(1, Pos(0, 0), Pos(1, 0), Pos(2, 0));

			if (range.satRange[i].system == GPS)
			{
				BLH.XYZtoBLH(R_WGS84, F_WGS84);
			}
			else
			{
				BLH.XYZtoBLH(R_CGS2K, F_CGS2K);
			}

			BLH.CompSatElAz(Pos_0, range.SatMidRes[i].SatPos);
			range.SatMidRes[i].Elevation = BLH.Elev;
			range.SatMidRes[i].Azimuth = BLH.Azim;

			//Tropospheric Correction
			ErrorCorrection Hop(BLH.H, range.SatMidRes[i].Elevation);
			range.SatMidRes[i].TropCorr = Hop.Hopfield();

			//BDS TGD Correction
			if (range.satRange[i].system == BDS)
			{
				double tgd;
				bool weak = true;
				for (int j = 0; j < BDSEph.size(); ++j)
				{
					if (range.satRange[i].PRN == BDSEph[j].PRN)
					{
						weak = false;
						tgd = BDSEph[j].TGD1;
					}
					else
					{
						continue;
					}
				}
				if (!weak)
				{
					range.SatMidRes[i].Tgd1 = (FG1_BDS * FG1_BDS * tgd) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
				}
				else
				{
					range.SatMidRes[i].Valid = false;
				}
			}

			//Matrix Assignment
			Matrix dPos(3, 1);
			dPos.MatrixSubtraction(RotaResultPos, Pos);
			double d = dPos.Magnitude();
			
			for (int j = 0; j < 3; ++j)
			{
				B(index - 1, j) = (Pos(j, 0) - RotaResultPos(j, 0)) / d;
			}

			if (range.satRange[i].system == GPS)
			{
				B(index - 1, 3) = 1;
				B(index - 1, 4) = 0;
				l(index - 1, 0) = range.ComObs[i].PIF- (d + dt[0] - C_Light * range.SatMidRes[i].SatClkOft + range.SatMidRes[i].TropCorr + C_Light * range.SatMidRes[i].Tgd1);
				GPScount++;
			}
			else
			{
				B(index - 1, 3) = 0;
				B(index - 1, 4) = 1;
				l(index - 1, 0) = range.ComObs[i].PIF - (d + dt[1] - C_Light * range.SatMidRes[i].SatClkOft + range.SatMidRes[i].TropCorr + C_Light * range.SatMidRes[i].Tgd1);
				BDScount++;
			}
		}
 
		if (GPScount == 0 || BDScount == 0)
		{
			paraNum = 4;
		}
		else
		{
			paraNum = 5;
		}

		//Least Squares Estimation
		Matrix N_xx(5, 5),N_BB(5,5), B_T(5, ValidSat), tempB(5, ValidSat), W(5, 1);
		B_T.MatrixTransposition(B);

		tempB.MatrixMultiplication(B_T, P);
		N_BB.MatrixMultiplication(tempB, B);
		W.MatrixMultiplication(tempB, l);

		//Matrix Reconstruction
		Matrix tran_N_BB(4, 4), tran_W(4, 1), tran_N_xx(4, 4);
		bool tran = false;
		if (GPScount == 0)
		{
			tran = true;
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					tran_N_BB(i, j) = N_BB(i, j);
				}
				tran_N_BB(i, 3) = N_BB(i, 4);
				tran_N_BB(3, i) = N_BB(4, i);
				tran_W(i, 0) = W(i, 0);
			}
			tran_N_BB(3, 3) = N_BB(4, 4);

			tran_W(3, 0) = W(4, 0);
		}

		if (BDScount == 0)
		{
			tran = true;
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					tran_N_BB(i, j) = N_BB(i, j);
				}
				tran_W(i, 0) = W(i, 0);
			}
		}

		Matrix x(paraNum, 1);
		if (!tran)
		{
			N_xx.InverseMatrix(N_BB);
			x.MatrixMultiplication(N_xx, W);
		}
		else
		{
			tran_N_xx.InverseMatrix(tran_N_BB);
			x.MatrixMultiplication(tran_N_xx, tran_W);
		}

		//Results of Iterative Calculation
		for (int i = 0; i < 3; ++i)
		{
			Pos_0[i] += x(i, 0);
		}

		if (GPScount == 0)
		{
			dt[1] += x(3, 0);
		}
		else if (BDScount == 0)
		{
			dt[0] += x(3, 0);
		}
		else
		{
			dt[0] += x(3, 0);
			dt[1] += x(4, 0);
		}

		Iterator++;

		mag = x.Magnitude();

		//Precision Evaluation
		if ((mag < 0.01) || (Iterator > 100))
		{
			Matrix tran_B(ValidSat, 4);
			if (GPScount == 0)
			{
				for (int i = 0; i < ValidSat; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						tran_B(i, j) = B(i, j);
					}
					tran_B(i, 3) = B(i, 4);
				}
			}
			
			if (BDScount == 0)
			{
				for (int i = 0; i < ValidSat; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						tran_B(i, j) = B(i, j);
					}
				}
			}

			Matrix Bx(ValidSat, 1), v(ValidSat, 1), v_T(1, ValidSat), v_temp(1, ValidSat), sigmaMat(1, 1);
			if (!tran)
			{
				Bx.MatrixMultiplication(B, x);
				PDOP = sqrt(N_xx(0, 0) + N_xx(1, 1) + N_xx(2, 2));
			}
			else
			{
				Bx.MatrixMultiplication(tran_B, x);
				PDOP = sqrt(tran_N_xx(0, 0) + tran_N_xx(1, 1) + tran_N_xx(2, 2));
			}

			v.MatrixSubtraction(Bx, l);
			v_T.MatrixTransposition(v);
			v_temp.MatrixMultiplication(v_T, P);
			sigmaMat.MatrixMultiplication(v_temp, v);
			sigma_0 = sqrt(sigmaMat(0, 0) / (ValidSat - paraNum));
		}
		
	} while ((mag >= 0.01) && (Iterator <= 100));

	if ((Pos_0[0] == 0.0) || (Pos_0[1] == 0.0) || (Pos_0[2] == 0.0))
	{
		std::cout << "Solution Unsuccessful!" << std::endl;
		return false;
	}

	//Computation Finished, Store the Results
	for (int i = 0; i < 3; ++i)
	{	
		PosRes.Pos[i] = Pos_0[i];
	}

	PosRes.SatNum = ValidSat;
	PosRes.Time = range.Time;
	PosRes.SigmaPos = sigma_0;
	PosRes.PDOP = PDOP;
	PosRes.GPSCount = GPScount;
	PosRes.BDSCount = BDScount;
	PosRes.SatNum = ValidSat;

    return true;
}

//Compute Receiver Velocity
bool SPP::SolveVelocity(const EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph)
{
	if (ValidSat < 4)
	{
		std::cout << "Insufficient observations, unable to compute!" << std::endl;
		return false;
	}
	Matrix B(ValidSat, 4), l(ValidSat, 1);
	int index = 0;

	if (range.SatMidRes.size() == 0)
	{
		std::cout << "Insufficient observations, unable to compute!" << std::endl;
		return false;
	}

	//Matrix Assignment
	for (int i = 0; i < range.SatMidRes.size(); ++i)
	{
		if ((!range.satRange[i].valid) || (!range.SatMidRes[i].Valid))
		{
			continue;
		}
		else
		{
			index++;
		}

		Matrix dPos(3, 1), Pos(3, 1), SatPos(3, 1);
		for (int j = 0; j < 3; ++j)
		{
			SatPos(j, 0) = range.SatMidRes[i].SatPos[j];
			Pos(j, 0) = PosRes.Pos[j];
		}

		dPos.MatrixSubtraction(SatPos, Pos);

		double d = dPos.Magnitude();
		
		for (int j = 0; j < 3; ++j)
		{
			B(index - 1, j) = (Pos(j, 0) - SatPos(j, 0)) / d;
		}
		B(index - 1, 3) = 1;

		double d_w = ((SatPos(0, 0) - Pos(0, 0)) * range.SatMidRes[i].SatVel[0] + (SatPos(1, 0) - Pos(1, 0)) * range.SatMidRes[i].SatVel[1] + (SatPos(2, 0) - Pos(2, 0)) * range.SatMidRes[i].SatVel[2]) / d;
		l(index - 1, 0) = range.satRange[i].D[0] - (d_w - C_Light * range.SatMidRes[i].SatClkSft);
	}

	Matrix P(ValidSat, ValidSat), B_T(4, ValidSat), B_temp(4, ValidSat), N_BB(4, 4), N_xx(4, 4), W(4, 1), Bx(ValidSat, 1), v(ValidSat, 1), v_T(1, ValidSat),v_temp(1,ValidSat), sigmaMat(1, 1), x(4, 1);
	for (int i = 0; i < ValidSat; ++i)
	{
		P(i, i) = 1;
	}

	//Least Squares Estimation
	B_T.MatrixTransposition(B);
	B_temp.MatrixMultiplication(B_T, P);
	N_BB.MatrixMultiplication(B_temp, B);
	N_xx.InverseMatrix(N_BB);
	W.MatrixMultiplication(B_temp, l);
	x.MatrixMultiplication(N_xx, W);

	//Precision Evaluation
	Bx.MatrixMultiplication(B, x);
	v.MatrixSubtraction(Bx, l);
	v_T.MatrixTransposition(v);
	v_temp.MatrixMultiplication(v_T, P);
	sigmaMat.MatrixMultiplication(v_temp, v);
	sigma_0 = sqrt(sigmaMat(0, 0) / (ValidSat - 4));

	//Computation Finished, Store the Results
	for (int i = 0; i < 3; ++i)
	{
		PosRes.Vel[i] = x(i, 0);
	}
	PosRes.SigmaVel = sigma_0;

	return true;
}

//Earth Rotation Correction
void SPP::RotationCorrection(Matrix& Pos, SATMIDRES& SATMID, const GNSSSys& system)
{
	Matrix SatPos(3, 1), SatVel(3, 1);
	for (int j = 0; j < 3; ++j)
	{
		SatPos(j, 0) = SATMID.SatPos[j];
		SatVel(j, 0) = SATMID.SatVel[j];
		Pos(j, 0) = Pos_0[j];
	}

	Matrix dPos(3, 1);
	dPos.MatrixSubtraction(SatPos, Pos);

	double dt0 = dPos.Magnitude() / C_Light;

	double rota = 0;
	if (system == GPS)
	{
		rota = Omega_WGS * dt0;
	}
	else
	{
		rota = Omega_BDS * dt0;
	}

	Matrix rotation(3, 3);
	rotation(0, 0) = cos(rota);
	rotation(0, 1) = sin(rota);
	rotation(1, 0) = -sin(rota);
	rotation(1, 1) = cos(rota);
	rotation(2, 2) = 1;

	Matrix RotaResultPos(3, 1), RotaResultVel(3, 1);
	RotaResultPos.MatrixMultiplication(rotation, SatPos);
	RotaResultVel.MatrixMultiplication(rotation, SatVel);

	for (int j = 0; j < 3; ++j)
	{
		SATMID.SatPos[j] = RotaResultPos(j, 0);
		SATMID.SatVel[j] = RotaResultVel(j, 0);
	}
}
