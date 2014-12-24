/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.test;

import com.blackrook.physics2d.Shape2D;

public class CollisionBody
{
	public Shape2D shape;
	public double x;
	public double y;
	public double vx;
	public double vy;
	public double rotation;
	
	public CollisionBody(Shape2D shape)
	{
		this.shape = shape;
	}
	
}
