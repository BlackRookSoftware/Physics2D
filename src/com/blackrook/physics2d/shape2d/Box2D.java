/*******************************************************************************
 * Copyright (c) 2014 - 2015 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.shape2d;

import com.blackrook.physics2d.Shape2D;

/**
 * An axis-aligned bounding box.
 * Body rotation does not affect AABBs - they are always aligned to the primary axes.
 * @author Matthew Tropiano
 */
public class Box2D implements Shape2D
{
	/** Box half width. */
	protected float halfWidth;
	/** Box half height. */
	protected float halfHeight;

	/**
	 * Creates a new axis-aligned bounding box.
	 * @param halfWidth the box's half-width.
	 * @param halfHeight the box's half-height.
	 */
	public Box2D(float halfWidth, float halfHeight)
	{
		this.halfWidth = halfWidth;
		this.halfHeight = halfHeight;
	}
	
	@Override
	public double getHalfWidth()
	{
		return halfWidth;
	}

	@Override
	public double getHalfHeight()
	{
		return halfHeight;
	}

	@Override
	public double getRadius()
	{
		return (float)Math.sqrt(getSquareRadius());
	}

	@Override
	public double getSquareRadius()
	{
		return halfWidth*halfWidth + halfHeight*halfHeight;
	}

	@Override
	public boolean useRadius()
	{
		return false;
	}

	@Override
	public String toString()
	{
		return "AABB[hw:"+halfWidth+", hh:"+halfHeight+"]";
	}
	
}
