/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.shape2d;

import com.blackrook.physics2d.Shape2D;

/**
 * Describes a circle shape.
 * Body rotation does not affect Circles for reasons that should be obvious.
 * @author Matthew Tropiano
 */
public class Circle implements Shape2D
{
	/** Circle radius. */
	protected float radius;

	/**
	 * Creates a new circle.
	 */
	public Circle(float radius)
	{
		this.radius = radius;
	}
	
	@Override
	public double getRadius()
	{
		return radius;
	}

	@Override
	public double getSquareRadius()
	{
		return radius * radius;
	}

	@Override
	public boolean useRadius()
	{
		return true;
	}

	@Override
	public double getHalfHeight()
	{
		return radius;
	}

	@Override
	public double getHalfWidth()
	{
		return radius;
	}

	@Override
	public String toString()
	{
		return "Circle[rad:"+radius+"]";
	}

}
