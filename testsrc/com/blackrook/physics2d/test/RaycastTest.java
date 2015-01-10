/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.test;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.event.MouseInputListener;

import com.blackrook.commons.logging.Logger;
import com.blackrook.commons.logging.LoggingFactory;
import com.blackrook.commons.math.RMath;
import com.blackrook.commons.math.geometry.Line2D;
import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.physics2d.Collision2D;
import com.blackrook.physics2d.Physics2DUtils;
import com.blackrook.physics2d.Shape2D;
import com.blackrook.physics2d.shape2d.Box2D;
import com.blackrook.physics2d.shape2d.Circle;
import com.blackrook.physics2d.shape2d.Polygon;

public class RaycastTest {

	public static Collision2D<CollisionBody> c2d;
	public static Line2D line = new Line2D();
	public static CollisionBody cbTarg;
	public static CollisionModel model = new CollisionModel(); 
	static final Logger logger = LoggingFactory.createConsoleLoggerFor(RaycastTest.class);

	
	public static void main(String[] args)
	{
		Shape2D[] shapes = {
			new Circle(50),
			new Box2D(50, 50),
			new Polygon(new Point2D[]{
				new Point2D(-40,  15),
				new Point2D(0,    40),
				new Point2D(40,   15),
				new Point2D(20,  -40),
				new Point2D(-20, -40),
			}),
			new Polygon(new Point2D[]{
				new Point2D(-20, 20),
				new Point2D(20, 20),
				new Point2D(20, -20),
				new Point2D(-20, -20)
			})
		};
		
		cbTarg = new CollisionBody(shapes[1]);
		
		c2d = new Collision2D<CollisionBody>();
		c2d.target = cbTarg;
		makeTestWindow(c2d, "Test");
	}

	
	private static JFrame makeTestWindow(Collision2D<CollisionBody> c2d, String name)
	{
		JFrame out = new JFrame(name);
		out.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		out.add(new TestCanvas(c2d));
		out.setResizable(true);
		out.pack();
		out.setVisible(true);
		return out;
	}

	private static class TestCanvas extends Canvas implements MouseInputListener
	{
		private static final long serialVersionUID = -7369670472224047283L;
		
		Collision2D<CollisionBody> c2d;
		Point2D tp1;
		Point2D tp2;
		BufferedImage bi;
		
		TestCanvas(Collision2D<CollisionBody> c2d)
		{
			this.c2d = c2d;
			tp1 = new Point2D();
			tp2 = new Point2D();
			setPreferredSize(new Dimension(640,480));
			addMouseMotionListener(this);
			addMouseListener(this);
			tc();
		}
		
		public void tc()
		{
			long n = System.nanoTime();
			boolean b = Physics2DUtils.testRaycastCollision(model, c2d, line, cbTarg);
			c2d.calcNanos = System.nanoTime() - n;
			logger.info(b+" "+"IV: "+c2d.incidentVector+" IP: "+c2d.incidentPoint+" "+c2d.method+" AXES: "+c2d.axisCount+" "+c2d.calcNanos+"ns");
		}

		public void update(Graphics g)
		{
			Graphics2D g2d = (Graphics2D)g;
			if (bi == null || bi.getWidth() != getWidth() || bi.getHeight() != getHeight())
				bi = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
			drawToImage(bi);
			g2d.drawImage(bi, null, 0, 0);
		}
		
		public void drawToImage(BufferedImage bi)
		{
			Graphics2D g2d = bi.createGraphics();
			g2d.setColor(Color.BLACK);
			g2d.fillRect(0, 0, getWidth(), getHeight());
			g2d.scale(1, -1);
			g2d.translate(0, -getHeight());
			
			g2d.setColor(Color.RED);
			drawShape(g2d, cbTarg.shape, cbTarg.x, cbTarg.y, cbTarg.vx, cbTarg.vy, cbTarg.rotation);

			g2d.setColor(Color.GRAY);
			drawScan(g2d, line);

			g2d.setColor(Color.WHITE);
			drawIncident(g2d, c2d);
			g2d.dispose();
		}
		
		public void drawScan(Graphics2D g2d, Line2D line)
		{
			double x = getWidth()/2;
			double y = getHeight()/2;
			int ix0 = (int)(x+line.pointA.x);
			int ix1 = (int)(x+line.pointB.x);
			int iy0 = (int)(y+line.pointA.y);
			int iy1 = (int)(y+line.pointB.y);
			g2d.drawLine(ix0, iy0, ix1, iy1);
		}
		
		public void drawIncident(Graphics2D g2d, Collision2D<CollisionBody> c2d)
		{
			double x = getWidth()/2;
			double y = getHeight()/2;
			int ix0 = (int)(x+c2d.incidentPoint.x);
			int ix1 = (int)(x+c2d.incidentPoint.x+c2d.incidentVector.x);
			int iy0 = (int)(y+c2d.incidentPoint.y);
			int iy1 = (int)(y+c2d.incidentPoint.y+c2d.incidentVector.y);
			g2d.drawLine(ix0, iy0, ix1, iy1);
			g2d.fillOval((int)(c2d.incidentPoint.x-2+x), (int)(c2d.incidentPoint.y-2+y), 4, 4);

		}
		
		public void drawShape(Graphics2D g2d, Shape2D s, double x, double y, double vx, double vy, double rotation)
		{
			int ix = (int)x + getWidth()/2;
			int iy = (int)y + getHeight()/2;
			int ir = (int)s.getRadius();
			int ihw = (int)s.getHalfWidth();
			int ihh = (int)s.getHalfHeight();
			int ivx = (int)vx;
			int ivy = (int)vy;
			if (s instanceof Circle)
			{
				g2d.drawOval(ix-ir, iy-ir, ir*2, ir*2);
				if (vx != 0.0 || vy != 0.0)
					g2d.drawOval(ix-ir-ivx, iy-ir-ivy, ir*2, ir*2);
			}
			else if (s instanceof Box2D)
			{
				g2d.drawRect(ix-ihw, iy-ihh, ihw*2, ihh*2);
				if (vx != 0.0 || vy != 0.0)
					g2d.drawRect(ix-ihw-ivx, iy-ihh-ivy, ihw*2, ihh*2);
			}
			else if (s instanceof Polygon)
			{
				Polygon p = (Polygon)s;
				Point2D[] pts = p.getPoints();
				for (int n = 0; n < pts.length; n++)
				{
					tp1.set(pts[n]);
					tp1.rotateZ((float)RMath.degToRad(rotation));
					tp2.set(pts[(n+1)%pts.length]);
					tp2.rotateZ((float)RMath.degToRad(rotation));
					g2d.drawLine((int)tp1.x+ix, (int)tp1.y+iy, (int)tp2.x+ix,(int)tp2.y+iy);
				}
				if (vx != 0.0 || vy != 0.0)
				{
					for (int n = 0; n < pts.length; n++)
					{
						tp1.set(pts[n]);
						tp1.rotateZ((float)RMath.degToRad(rotation));
						tp2.set(pts[(n+1)%pts.length]);
						tp2.rotateZ((float)RMath.degToRad(rotation));
						g2d.drawLine((int)tp1.x+ix-ivx, (int)tp1.y+iy-ivy, (int)tp2.x+ix-ivx,(int)tp2.y+iy-ivy);
					}
				}
			}
		}

		@Override
		public void mouseDragged(MouseEvent e) {
			CollisionBody b = cbTarg;
			b.x = e.getX()-(getWidth()/2);
			b.y = -(e.getY()-(getHeight()/2));
			tc();
			repaint();
		}

		@Override
		public void mouseMoved(MouseEvent e) {
			line.pointB.x = e.getX()-(getWidth()/2);
			line.pointB.y = -(e.getY()-(getHeight()/2));
			tc();
			repaint();
		}

		@Override
		public void mouseClicked(MouseEvent e){
		}

		@Override
		public void mouseEntered(MouseEvent e) {
		}

		@Override
		public void mouseExited(MouseEvent e) {
		}

		@Override
		public void mousePressed(MouseEvent e) {
		}

		@Override
		public void mouseReleased(MouseEvent e) {
		}
		
	}

}
