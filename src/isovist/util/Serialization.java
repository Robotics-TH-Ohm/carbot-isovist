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
			elem.setAttribute("grid-y", String.valueOf(x));
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
}
