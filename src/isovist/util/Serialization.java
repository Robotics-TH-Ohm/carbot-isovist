package isovist.util;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.OutputKeys;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.w3c.dom.Node;
import java.io.File;

import isovist.model.*;
import isovist.model.features.*;

public class Serialization
{
	public static Document isovistGridToDocument(IsovistGrid grid) throws Exception
	{
		DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
		Document doc = builder.newDocument();
    Element root = doc.createElement("isovist-map");
		root.setAttribute("grid-size", String.valueOf(grid.getGridSize()));
    doc.appendChild(root);

		// For every isovist (= cell) in map!
		grid.processAll((storage, x, y) -> {
			Cell cell = storage.get(x, y);
			if (cell == null || cell.getIsovist() == null) return;

			Isovist iso = cell.getIsovist();
			double[] features = iso.getFeatures();

			Element elem = doc.createElement("isovist");
			elem.setAttribute("grid-x", String.valueOf(x));
			elem.setAttribute("grid-y", String.valueOf(y));
			elem.setAttribute("cell", cell.toString());
			for (int i = 0; i < features.length; ++i)
			{
				Element feat = doc.createElement(Isovist.features[i].getName());
				feat.appendChild(doc.createTextNode(String.valueOf(features[i])));
				elem.appendChild(feat);
			}
			root.appendChild(elem);
		});

		return doc;
	}

	public static void documentToFile(Document document, String path) throws Exception
	{
		TransformerFactory factory = TransformerFactory.newInstance();
		factory.setAttribute("indent-number", 2);

		Transformer transformer = factory.newTransformer();
		transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
		transformer.setOutputProperty(OutputKeys.INDENT, "yes");

		DOMSource source = new DOMSource(document);
		StreamResult result = new StreamResult(path);
		transformer.transform(source, result);
	}

	public static Document readDocumentFromFile(String path) throws Exception
	{
		File xmlFile = new File(path);

		// Create a DocumentBuilder
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = factory.newDocumentBuilder();

		// Parse the XML file
		Document document = builder.parse(xmlFile);
		return document;
	}

	public static IsovistGrid isovistGridFromDocument(Document doc) throws Exception
	{
		Element root = doc.getDocumentElement();

		double size = Double.parseDouble(root.getAttribute("grid-size"));
		IsovistGrid grid = new IsovistGrid(size);

		NodeList isovists = doc.getElementsByTagName("isovist");
		for (int i = 0; i < isovists.getLength(); ++i)
		{
			Element elem = (Element)isovists.item(i);
			int x = Integer.parseInt(elem.getAttribute("grid-x"));
			int y = Integer.parseInt(elem.getAttribute("grid-y"));
			System.out.println("Loading isovist at " + x + "/" + y);

			Cell cell = new Cell((byte)Integer.parseInt(elem.getAttribute("cell"), 2));

			double[] featureVec = new double[Isovist.features.length];
			for (int f = 0; f < featureVec.length; ++f)
			{
				Node node = elem.getElementsByTagName(Isovist.features[f].getName()).item(0);
				String val = node.getFirstChild().getNodeValue();
				featureVec[f] = Double.parseDouble(val);
			}

			Isovist isovist = new Isovist(featureVec);
			grid.set(x, y, cell.withIsovist(isovist));
		}

		return grid;
	}
}
